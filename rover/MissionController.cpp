// MissionController.cpp
#include "MissionController.h"
#include "Logger.h"
#include <cmath>
#include <algorithm>

MissionController::MissionController(Robot& robot, UartDriver& uartDriver)
    : m_robot(robot)
    , m_uartDriver(uartDriver)
{

    // Настраиваем callback для приёма пакетов от Arduino
    m_uartDriver.setPacketCallback([this](const commands::CommandPacket& packet)
                                   {
                                       handleIncomingPacket(packet);
                                   });

    // Запускаем поток чтения UART (если UartDriver его не имеет)
    // Если UartDriver сам управляет потоком, то эта строка не нужна
    m_uartReadingThread.emplace([this](std::stop_token token)
                                {
                                    while (!token.stop_requested())
                                    {
                                        m_uartDriver.processIncomingData();
                                        std::this_thread::sleep_for(std::chrono::milliseconds(5));
                                    }
                                });
}

MissionController::~MissionController()
{
    // Останавливаем миссию
    stopMission();

    // Останавливаем поток чтения
    if (m_uartReadingThread.has_value())
    {
        m_uartReadingThread->request_stop();
        m_uartReadingThread->join();
    }
}

void MissionController::startMission(const std::vector<mathLib::Vec2>& waypoints)
{
    if (waypoints.empty())
    {
        log(LogLevel::Warn, "MissionController: cannot start mission with empty waypoints");
        return;
    }

    // Останавливаем предыдущую миссию если есть
    stopMission();

    // Сохраняем путь
    m_plannedPath = waypoints;
    m_nextWaypointIndex = 0;
    m_isMissionActive = true;
    m_currentMode = ExecutionMode::WAYPOINT;

    // Очищаем буфер
    {
        std::lock_guard<std::mutex> lock(m_waypointBufferMutex);
        while (!m_waypointBuffer.empty()) m_waypointBuffer.pop();
    }

    // Заполняем первые точки
    refillWaypointBuffer();

    log(LogLevel::Info, "MissionController: mission started with " +
        std::to_string(waypoints.size()) + " waypoints");
}

void MissionController::stopMission()
{
    if (!m_isMissionActive) return;

    m_isMissionActive = false;
    m_currentMode = ExecutionMode::IDLE;

    // Отправляем команду остановки
    commands::CommandPacket stopCommand;
    stopCommand.type = commands::CmdType::STOP;
    stopCommand.cmdId = 0; // Emergency ID
    sendCommand(stopCommand);

    // Очищаем путь и буфер
    {
        std::lock_guard<std::mutex> lock(m_waypointBufferMutex);
        while (!m_waypointBuffer.empty()) m_waypointBuffer.pop();
    }
    m_plannedPath.clear();
    m_nextWaypointIndex = 0;

    log(LogLevel::Info, "MissionController: mission stopped");
}

void MissionController::update()
{
    if (!m_isMissionActive) return;

    // Получаем актуальную позицию (атомарно)
    auto currentPose = m_robot.getPose();

    // Режим WAYPOINT: управляем конвейером
    if (m_currentMode == ExecutionMode::WAYPOINT)
    {
        checkModeTransition(); // Проверяем, не пора ли в Velocity Mode

        // Проверяем достижение текущей waypoint
        mathLib::Vec2 currentWaypoint;
        {
            std::lock_guard<std::mutex> lock(m_waypointBufferMutex);
            if (!m_waypointBuffer.empty())
            {
                currentWaypoint = m_waypointBuffer.front();
            }
            else
            {
                return; // Буфер пуст, ждём пополнения
            }
        }

        // Если waypoint достигнут
        float tolerance = (m_nextWaypointIndex == m_plannedPath.size()) ?
            FINAL_WAYPOINT_TOLERANCE : DEFAULT_WAYPOINT_TOLERANCE;

        if (isWaypointReached(currentWaypoint, tolerance))
        {
            log(LogLevel::Debug, "MissionController: waypoint reached, index=" +
                std::to_string(m_nextWaypointIndex - m_waypointBuffer.size()));

            // Удаляем из буфера
            {
                std::lock_guard<std::mutex> lock(m_waypointBufferMutex);
                m_waypointBuffer.pop();
            }

            // Заполняем новыми точками
            refillWaypointBuffer();
        }
    }
    // Режим VELOCITY: Pi управляет напрямую
    else if (m_currentMode == ExecutionMode::VELOCITY)
    {
        // Здесь будет логика Visual Servoing
        // Пока просто логируем
        float distance = std::hypot(
            m_velocityTarget.x - currentPose.position.x,
            m_velocityTarget.y - currentPose.position.y
        );
        m_distanceToGoal = distance;

        log(LogLevel::Debug, "MissionController: velocity mode, distance to goal=" +
            std::to_string(distance));
    }
}

void MissionController::refillWaypointBuffer()
{
    std::lock_guard<std::mutex> lock(m_waypointBufferMutex);

    // Пока буфер не полон и есть точки в пути
    while (m_waypointBuffer.size() < ARDUINO_WAYPOINT_BUFFER_SIZE &&
           m_nextWaypointIndex < m_plannedPath.size())
    {

        const auto& waypoint = m_plannedPath[m_nextWaypointIndex];
        m_waypointBuffer.push(waypoint);

        // Формируем команду
        commands::CommandPacket command;
        command.type = commands::CmdType::GOTO;
        command.cmdId = static_cast<uint8_t>(m_nextWaypointIndex + 1);
        command.waypointId = static_cast<uint8_t>(m_nextWaypointIndex + 1);
        command.goto_data.x = waypoint.x;
        command.goto_data.y = waypoint.y;
        command.goto_data.theta = calculateTargetOrientation(m_nextWaypointIndex);
        command.goto_data.linearSpeed = 0.3f;
        command.goto_data.angularSpeed = 0.5f;
        command.goto_data.tolerance = (m_nextWaypointIndex + 1 == m_plannedPath.size()) ?
            FINAL_WAYPOINT_TOLERANCE : DEFAULT_WAYPOINT_TOLERANCE;

        sendCommand(command);

        log(LogLevel::Info, "MissionController: sent waypoint " +
            std::to_string(m_nextWaypointIndex + 1) +
            " (" + std::to_string(waypoint.x) + ", " + std::to_string(waypoint.y) + ")");

        ++m_nextWaypointIndex;
    }
}

float MissionController::calculateTargetOrientation(size_t waypointIndex) const
{
    if (waypointIndex + 1 >= m_plannedPath.size())
    {
        return 0.0f; // Финальная точка: ориентация не важна
    }

    // Смотрим на следующую точку
    const auto& current = m_plannedPath[waypointIndex];
    const auto& next = m_plannedPath[waypointIndex + 1];

    return std::atan2(next.y - current.y, next.x - current.x);
}

void MissionController::checkModeTransition()
{
    // Вычисляем расстояние до финальной цели
    auto currentPose = m_robot.getPose();
    const auto& finalGoal = m_plannedPath.back();

    float distanceToGoal = std::hypot(
        finalGoal.x - currentPose.position.x,
        finalGoal.y - currentPose.position.y
    );
    m_distanceToGoal = distanceToGoal;

    // Если близко — переключаемся
    if (distanceToGoal < VELOCITY_MODE_DISTANCE_THRESHOLD)
    {
        enableVelocityMode(finalGoal);
    }
}

void MissionController::enableVelocityMode(const mathLib::Vec2& targetPosition)
{
    if (m_currentMode == ExecutionMode::VELOCITY) return;

    m_velocityTarget = targetPosition;
    m_currentMode = ExecutionMode::VELOCITY;

    // Останавливаемся перед переключением
    commands::CommandPacket stopCommand;
    stopCommand.type = commands::CmdType::STOP;
    stopCommand.cmdId = 255; // Специальный ID для остановки
    sendCommand(stopCommand);

    log(LogLevel::Info, "MissionController: switched to VELOCITY mode, target=" +
        std::to_string(targetPosition.x) + ", " + std::to_string(targetPosition.y));
}

bool MissionController::isWaypointReached(const mathLib::Vec2& waypoint, float tolerance) const
{
    auto currentPose = m_robot.getPose();
    float deltaX = waypoint.x - currentPose.position.x;
    float deltaY = waypoint.y - currentPose.position.y;
    return std::hypot(deltaX, deltaY) < tolerance;
}

void MissionController::handleIncomingPacket(const commands::CommandPacket& packet)
{
    switch (packet.type)
    {
    case commands::CmdType::REACHED:
        log(LogLevel::Debug, "MissionController: Arduino confirmed waypoint " +
            std::to_string(packet.waypointId));
        break;

    case commands::CmdType::ACK:
        log(LogLevel::Warn, "MissionController: command acknowledged ID=" +
            std::to_string(packet.cmdId));
        break;

    default:
        log(LogLevel::Warn, "MissionController: unexpected packet type " +
            std::to_string(static_cast<int>(packet.type)));
        break;
    }
}

void MissionController::sendCommand(const commands::CommandPacket& command)
{
    // Рассчитываем контрольную сумму (XOR всех байт)
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&command);
    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(command) - 1; ++i)
    {
        checksum ^= bytes[i];
    }

    // Копируем команду и дописываем checksum
    commands::CommandPacket cmdWithChecksum = command;
    cmdWithChecksum.checksum = checksum;

    if (!m_uartDriver.sendPacket(cmdWithChecksum))
    {
        log(LogLevel::Error, "MissionController: failed to send command");
    }
}