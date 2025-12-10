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
    enum class ExecutionMode : uint8_t
    {
        IDLE,           // Нет активной миссии
        WAYPOINT,       // Движение к промежуточным точкам (Arduino автопилот)
        VELOCITY,       // Донаведение по камере (Pi управляет напрямую)
        DOCKED          // Миссия завершена
    };


    explicit MissionController(Robot& robot, UartDriver& uartDriver);

    // Деструктор — останавливает миссию и потоки
    ~MissionController();

    // Запрет копирование
    MissionController(const MissionController&) = delete;
    MissionController& operator=(const MissionController&) = delete;

    // Запускает новую миссию по заданному пути
    //  waypoints Вектор ключевых точек (от A*)
    void startMission(const std::vector<mathLib::Vec2>& waypoints);

    // Немедленно останавливает миссию
    void stopMission();

    // Главный цикл обновления (вызывается каждые 10-50 мс)
    void update();

    // Переключает в режим донаведения (вызывается при EventType::TargetInView)
    //  targetPosition Позиция цели в мире (метры)
    void enableVelocityMode(const mathLib::Vec2& targetPosition);

    // Обработчик пакетов от Arduino (вызывается из UART потока)
    void handleIncomingPacket(const commands::CommandPacket& packet);

    // Возвращает текущий режим выполнения
    ExecutionMode getCurrentMode() const { return m_currentMode.load(); }

    // Возвращает флаг активности миссии
    bool isMissionActive() const { return m_isMissionActive.load(); }

    // Отправка команды на микроконтроллер
    void sendCommand(const commands::CommandPacket& command);

private:

    static bool verifyChecksum(const commands::CommandPacket& packet);

    // Проверяет, достигнута ли текущая waypoint
    //  waypoint Позиция waypoint'а
    //  tolerance Допустимое отклонение (метры)
    bool isWaypointReached(const mathLib::Vec2& waypoint, float tolerance) const;

    // Заполняет буфер Arduino новыми waypoint'ами (конвейер)
    void refillWaypointBuffer();

    // Вычисляет ориентацию (theta) по направлению к следующей точке
    float calculateTargetOrientation(size_t waypointIndex) const;

    // Проверяет, нужно ли переключаться в Velocity Mode
    void checkModeTransition();

    // Повторная отправка команды (заглушка)
    void resendCommand(uint8_t cmdId) {}

    // Константы
    static constexpr size_t ARDUINO_WAYPOINT_BUFFER_SIZE = 3; // Максимум 3 точки в буфере
    static constexpr float VELOCITY_MODE_DISTANCE_THRESHOLD = 1.5f; // метров
    static constexpr float DEFAULT_WAYPOINT_TOLERANCE = 0.01f; // 1 см
    static constexpr float FINAL_WAYPOINT_TOLERANCE = 0.02f; // 2 см

    // Ссылки на внешние компоненты
    Robot& m_robot;
    UartDriver& m_uartDriver;

    std::chrono::steady_clock::time_point m_lastHeartbeat;

    // Внутреннее состояние
    std::atomic<ExecutionMode> m_currentMode{ ExecutionMode::IDLE };
    std::atomic<bool> m_isMissionActive{ false };

    // Весь спланированный путь
    std::vector<mathLib::Vec2> m_plannedPath;
    std::atomic<size_t> m_nextWaypointIndex{ 0 }; // Следующая точка для отправки в Arduino

    // Буфер waypoint'ов для Arduino (FIFO очередь)
    mutable std::mutex m_waypointBufferMutex;
    std::queue<mathLib::Vec2> m_waypointBuffer;

    // Цель для Velocity Mode
    mathLib::Vec2 m_velocityTarget;
    std::atomic<float> m_distanceToGoal{ 0.0f };

    // Поток для чтения UART
    std::optional<std::jthread> m_uartReadingThread;
};