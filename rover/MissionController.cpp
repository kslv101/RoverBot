#include "MissionController.h"
#include "Logger.h"
#include <cmath>
#include <thread>
#include <chrono>
#include <cstring> // для memset

MissionController::MissionController(Robot& robot, UartDriver& uartDriver, EventQueue& eventQueue)
    : m_robot(robot), m_uartDriver(uartDriver), m_eventQueue(eventQueue)
{
    m_uartDriver.setPacketCallback([this](const uint8_t* data, size_t size) 
        {
        handleIncomingPacket(data, size);
        });

    m_uartThread.emplace([this](std::stop_token token) 
        {
        while (!token.stop_requested()) {
            m_uartDriver.processIncomingData();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        });
}

MissionController::~MissionController() 
{
    if (m_uartThread.has_value() && m_uartThread->joinable()) 
    {
        m_uartThread->request_stop();
        m_uartThread->join();
    }
}

void MissionController::startMission(const std::vector<mathLib::Vec2>& waypoints) 
{
    if (waypoints.empty()) 
    {
        log(LogLevel::Error, "MissionController: Cannot start mission with empty waypoints");
        return;
    }

    stopMission();

    m_segments = buildSegments(waypoints);
    if (m_segments.empty()) 
    {
        log(LogLevel::Error, "MissionController: Failed to build segments from waypoints");
        return;
    }

    log(LogLevel::Info, "MissionController: Built " + std::to_string(m_segments.size()) + " segments from " + std::to_string(waypoints.size()) + " waypoints");

    m_nextSegmentIndex = 0;
    m_currentSegmentId = 1;
    m_isActive = true;
    m_state = ExecutionState::EXECUTING;
    m_waitingForAck = false;
    m_currentSegmentWaiting = 0;

    sendNextSegment();

    log(LogLevel::Info, "MissionController: Mission started");
}

void MissionController::stopMission() 
{
    if (!m_isActive) return;

    log(LogLevel::Info, "MissionController: Stopping mission");
    sendStopCommand();

    m_isActive = false;
    m_state = ExecutionState::IDLE;
    m_segments.clear();
    m_nextSegmentIndex = 0;
    m_waitingForAck = false;
    m_currentSegmentWaiting = 0;
}

void MissionController::emergencyStop() 
{
    log(LogLevel::Error, "MissionController: EMERGENCY STOP activated");
    sendEmergencyStopCommand();

    m_isActive = false;
    m_state = ExecutionState::EMERGENCY_STOP;
    m_segments.clear();
    m_nextSegmentIndex = 0;
    m_waitingForAck = false;
    m_currentSegmentWaiting = 0;
}

void MissionController::update() 
{
    if (!m_isActive || m_state != ExecutionState::EXECUTING) return;

    // Проверяем, не завершилась ли миссия
    if (m_nextSegmentIndex >= m_segments.size() && !m_waitingForAck) 
    {
        log(LogLevel::Info, "MissionController: Mission completed successfully");
        m_isActive = false;
        m_state = ExecutionState::COMPLETED;
        m_eventQueue.push(Event(EventType::DestinationReached));
    }
}

void MissionController::handleIncomingPacket(const uint8_t* data, size_t size) 
{
    // 4 байта (0xAA, segmentId, status, checksum)
    if (size != 4) 
    {
        log(LogLevel::Warn, "UART: Invalid packet size: " + std::to_string(size) +" (expected 4 bytes)");
        return;
    }

    if (data[0] != 0xAA) 
    {
        log(LogLevel::Warn, "UART: Invalid start byte: 0x" +std::to_string(static_cast<int>(data[0])));
        return;
    }

    // Проверка контрольной суммы
    uint8_t calculated = data[0] ^ data[1] ^ data[2];
    if (calculated != data[3]) 
    {
        log(LogLevel::Error, "UART: Checksum error! Calculated: 0x" + std::to_string(calculated) + ", Received: 0x" + std::to_string(static_cast<int>(data[3])));
        return;
    }

    uint8_t segmentId = data[1];
    bool success = (data[2] == 0x01);

    log(LogLevel::Info, "Arduino: Segment " + std::to_string(segmentId) + (success ? " completed successfully" : " failed"));

    handleSegmentAcknowledgment(segmentId, success);
}

void MissionController::handleSegmentAcknowledgment(uint8_t segmentId, bool success) 
{
    if (!m_isActive) return;

    // Проверяем, что это подтверждение для текущего сегмента
    if (!m_waitingForAck) 
    {
        log(LogLevel::Warn, "Unexpected acknowledgment for segment " + std::to_string(segmentId) + " (not waiting for ACK)");
        return;
    }

    if (segmentId != m_currentSegmentWaiting) 
    {
        log(LogLevel::Error, "Segment ID mismatch: expected " + std::to_string(m_currentSegmentWaiting) + ", got " + std::to_string(segmentId));
        emergencyStop();
        return;
    }

    m_waitingForAck = false;
    m_currentSegmentWaiting = 0;

    if (success) 
    {
        // Отправляем следующий сегмент (если есть)
        sendNextSegment();
    }
    else 
    {
        log(LogLevel::Error, "Segment " + std::to_string(segmentId) + " failed");
        emergencyStop();
    }
}

std::vector<MissionController::MovementSegment> MissionController::buildSegments(const std::vector<mathLib::Vec2>& waypoints) 
{
    std::vector<MovementSegment> segments;
    if (waypoints.size() < 2) return segments;

    auto currentPose = m_robot.getPose();
    mathLib::Vec2 currentPosition = currentPose.position;
    float currentAngle = currentPose.yaw;

    uint8_t segmentId = 1;

    for (size_t i = 0; i < waypoints.size(); ++i) 
    {
        const auto& target = waypoints[i];

        // Вычисление угла к цели
        float targetAngle = calculateAngleBetween(currentPosition, target);
        float angleDiff = normalizeAngle(targetAngle - currentAngle);

        // Если нужно повернуть, пушим команду
        if (std::abs(angleDiff) * 180.0 / M_PI > ANGLE_TOLERANCE)
        {
            int16_t angleValue = static_cast<int16_t>(angleDiff * 180.0 / M_PI * 10.0f);
            segments.push_back({ segmentId++, commands::MotionType::ROTATE, angleValue });
            currentAngle = targetAngle;
        }

        // Движение к точке
        float distance = std::hypot(target.x - currentPosition.x, target.y - currentPosition.y);
        if (distance > DISTANCE_TOLERANCE)
        {
            int16_t distanceMm = static_cast<int16_t>(distance * 100.0f);
            segments.push_back({ segmentId++, commands::MotionType::STRAIGHT, distanceMm });
            currentPosition = target;
        }
        else 
        {
            currentPosition = target;
        }
    }

    // Финальный поворот к направлению последней точки
    if (!waypoints.empty() && waypoints.size() > 1) 
    {
        const auto& lastPoint = waypoints.back();
        const auto& prevPoint = waypoints[waypoints.size() - 2];
        float finalAngle = calculateAngleBetween(prevPoint, lastPoint);
        float angleDiff = normalizeAngle(finalAngle - currentAngle);

        if (std::abs(angleDiff) > ANGLE_TOLERANCE) 
        {
            int16_t angleValue = static_cast<int16_t>(angleDiff * 10.0f);
            segments.push_back({ segmentId++, commands::MotionType::ROTATE, angleValue });
        }
    }

    return segments;
}

void MissionController::sendSegment(const MovementSegment& segment) 
{
    commands::MoveCommand cmd;
    memset(&cmd, 0, sizeof(cmd));

    cmd.header.startByte = 0xAA;
    cmd.header.commandType = commands::CommandType::MOVE;
    cmd.header.packetId = segment.segmentId;
    cmd.segmentId = segment.segmentId;
    cmd.motionType = segment.type;
    cmd.targetValue = segment.targetValue;

    // Расчёт контрольной суммы
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&cmd);
    cmd.checksum = 0;
    for (size_t i = 0; i < sizeof(commands::MoveCommand) - 1; i++) 
    {
        cmd.checksum ^= bytes[i];
    }

    if (!m_uartDriver.sendRaw(reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd))) 
    {
        log(LogLevel::Error, "UART: Failed to send segment " + std::to_string(segment.segmentId));
    }
}

void MissionController::sendNextSegment() 
{
    if (!m_isActive || m_state != ExecutionState::EXECUTING) return;
    if (m_waitingForAck) return; // Ожидание подтверждения предыдущего сегмента

    if (m_nextSegmentIndex >= m_segments.size()) 
    {
        log(LogLevel::Debug, "No more segments to send");
        return;
    }

    const auto& segment = m_segments[m_nextSegmentIndex];
    sendSegment(segment);

    // Устанавливка флага ожидания подтверждения
    m_waitingForAck = true;
    m_currentSegmentWaiting = segment.segmentId;
    m_nextSegmentIndex++;

    log(LogLevel::Info, "Sent segment " + std::to_string(segment.segmentId) +" (type=" + (segment.type == commands::MotionType::STRAIGHT ? "STRAIGHT" : "ROTATE") + ", value=" + std::to_string(segment.targetValue) + ")");
}

void MissionController::sendStopCommand() 
{
    struct 
    {
        uint8_t startByte;
        commands::CommandType commandType;
        uint8_t packetId;
        uint8_t checksum;
    } cmd;

    cmd.startByte = 0xAA;
    cmd.commandType = commands::CommandType::STOP;
    cmd.packetId = 0;

    cmd.checksum = cmd.startByte ^ static_cast<uint8_t>(cmd.commandType) ^ cmd.packetId;

    m_uartDriver.sendRaw(reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

void MissionController::sendEmergencyStopCommand() 
{
    struct 
    {
        uint8_t startByte;
        commands::CommandType commandType;
        uint8_t packetId;
        uint8_t checksum;
    } cmd;

    cmd.startByte = 0xAA;
    cmd.commandType = commands::CommandType::EMERGENCY_STOP;
    cmd.packetId = 0xFF;

    cmd.checksum = cmd.startByte ^ static_cast<uint8_t>(cmd.commandType) ^ cmd.packetId;

    m_uartDriver.sendRaw(reinterpret_cast<const uint8_t*>(&cmd), sizeof(cmd));
}

float MissionController::calculateAngleBetween(const mathLib::Vec2& from, const mathLib::Vec2& to) const 
{
    return std::atan2(to.y - from.y, to.x - from.x);
}

float MissionController::normalizeAngle(float angle) const 
{
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}