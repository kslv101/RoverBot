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

/// @brief Управляет выполнением миссии: конвейер waypoint и режим донаведения
class MissionController
{
public:
    /// @brief Режимы выполнения миссии
    enum class ExecutionMode : uint8_t
    {
        IDLE,           // Нет активной миссии
        WAYPOINT,       // Движение к промежуточным точкам (Arduino автопилот)
        VELOCITY,       // Донаведение по камере (Pi управляет напрямую)
        DOCKED          // Миссия завершена
    };

    /// @brief Конструктор
    /// @param robot Ссылка на объект робота (для доступа к позиции и пути)
    /// @param uartDriver Ссылка на UART драйвер (для связи с Arduino)
    explicit MissionController(Robot& robot, UartDriver& uartDriver);

    /// @brief Деструктор — останавливает миссию и потоки
    ~MissionController();

    // Запрещаем копирование
    MissionController(const MissionController&) = delete;
    MissionController& operator=(const MissionController&) = delete;

    /// @brief Запускает новую миссию по заданному пути
    /// @param waypoints Вектор ключевых точек (от A*)
    void startMission(const std::vector<mathLib::Vec2>& waypoints);

    /// @brief Немедленно останавливает миссию
    void stopMission();

    /// @brief Главный цикл обновления (вызывается каждые 10-50 мс)
    void update();

    /// @brief Переключает в режим донаведения (вызывается при EventType::TargetInView)
    /// @param targetPosition Позиция цели в мире (метры)
    void enableVelocityMode(const mathLib::Vec2& targetPosition);

    /// @brief Обработчик пакетов от Arduino (вызывается из UART потока)
    void handleIncomingPacket(const commands::CommandPacket& packet);

    /// @brief Возвращает текущий режим выполнения
    ExecutionMode getCurrentMode() const { return m_currentMode.load(); }

    /// @brief Возвращает флаг активности миссии
    bool isMissionActive() const { return m_isMissionActive.load(); }

private:
    /// @brief Отправляет команду в Arduino с расчётом контрольной суммы
    void sendCommand(const commands::CommandPacket& command);

    /// @brief Проверяет, достигнута ли текущая waypoint
    /// @param waypoint Позиция waypoint'а
    /// @param tolerance Допустимое отклонение (метры)
    bool isWaypointReached(const mathLib::Vec2& waypoint, float tolerance) const;

    /// @brief Заполняет буфер Arduino новыми waypoint'ами (конвейер)
    void refillWaypointBuffer();

    /// @brief Вычисляет ориентацию (theta) по направлению к следующей точке
    float calculateTargetOrientation(size_t waypointIndex) const;

    /// @brief Проверяет, нужно ли переключаться в Velocity Mode
    void checkModeTransition();

    // Константы
    static constexpr size_t ARDUINO_WAYPOINT_BUFFER_SIZE = 3; // Максимум 3 точки в буфере
    static constexpr float VELOCITY_MODE_DISTANCE_THRESHOLD = 1.5f; // метров
    static constexpr float DEFAULT_WAYPOINT_TOLERANCE = 0.05f; // 5 см
    static constexpr float FINAL_WAYPOINT_TOLERANCE = 0.02f; // 2 см

    // Ссылки на внешние компоненты
    Robot& m_robot;
    UartDriver& m_uartDriver;

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