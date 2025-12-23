// MissionController.h
#pragma once

#include "CommandTypes.h"
#include "Robot.h"
#include "UartDriver.h"
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <optional>

/// Управляет выполнением миссии: конвейер waypoint и режим донаведения
class MissionController
{
public:
    // Режимы выполнения миссии
    enum class ExecutionState {
        IDLE,           // Нет активной миссии
        PLANNING,       // Выполняется планирование
        EXECUTING,      // Исполняются сегменты
        COMPLETED,      // Миссия завершена
        FAILED,         // Миссия прервана с ошибкой
        EMERGENCY_STOP  // Аварийная остановка
    };


    explicit MissionController(Robot& robot, UartDriver& uartDriver);
    ~MissionController();

    // Запрет копирование
    MissionController(const MissionController&) = delete;
    MissionController& operator=(const MissionController&) = delete;

    // Запуск миссии с ключевыми точками
    void startMission(const std::vector<mathLib::Vec2>& waypoints);

    // Остановка миссии
    void stopMission();

    // Экстренная остановка
    void emergencyStop();

    // Обновление состояния 
    void update();

    // Обработка входящего пакета от Arduino
    void handleIncomingPacket(const uint8_t* data, size_t size);

    ExecutionState getCurrentState() const { return m_state.load(); }
private:
    // Структура сегмента движения
    struct MovementSegment {
        uint8_t segmentId;          // Уникальный ID сегмента (1-255)
        commands::MotionType type;  // STRAIGHT или ROTATE
        int16_t targetValue;        // мм для движения, градусы*10 для поворота
    };

    // Разбивка пути на сегменты
    std::vector<MovementSegment> buildSegments(const std::vector<mathLib::Vec2>& waypoints); 

    // Отправка одного сегмента на Arduino
    void sendSegment(const MovementSegment& segment); 

    // Отправка следующего сегмента
    void sendNextSegment(); 

    // Обработка подтверждения сегмента
    void handleSegmentAcknowledgment(uint8_t segmentId, bool success);

    // Отправка команды остановки
    void sendStopCommand();

    // Отправка команды экстренной остановки
    void sendEmergencyStopCommand();

    // Вспомогательные методы
    float calculateAngleBetween(const mathLib::Vec2& from, const mathLib::Vec2& to) const;
    float normalizeAngle(float angle) const;

    // Константы
    static constexpr float ANGLE_TOLERANCE = 5.0f;    // 5 градусов
    static constexpr float DISTANCE_TOLERANCE = 0.05f; // 5 см

    Robot& m_robot;
    UartDriver& m_uartDriver;

    // Состояние
    std::atomic<ExecutionState> m_state{ ExecutionState::IDLE };
    std::atomic<bool> m_isActive{ false };

    // Сегменты
    std::vector<MovementSegment> m_segments;
    std::atomic<size_t> m_nextSegmentIndex{ 0 };
    std::atomic<uint8_t> m_currentSegmentId{ 1 }; // Начинаем с 1

    std::atomic<bool> m_waitingForAck{ false };
    std::atomic<uint8_t> m_currentSegmentWaiting{ 0 }; // ID текущего сегмента в ожидании

    // Поток UART
    std::optional<std::jthread> m_uartThread;
};