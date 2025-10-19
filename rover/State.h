#pragma once
#include <cstdint>
#include <string>

// Возможные состояния робота
enum class State : std::uint8_t
{
    Init, // инициализация
    Idle, // ожидание команды оператора (цели)
    Checkup, // проверка

    PlanningRoute, // планирование маршрута
    MovingToTarget, //едем по пути, проверяем препятствия
    AvoidingObstacle, //перепланируем и объезжаем или ждём
    ApproachingTarget,// приближение к цели (донаведение по камере)

    Wait, //ожидание
    EmergencyStop, // экстренная остановка

    RealSense,
    ReadControl,
    SendControl
};

std::string toString(State state);
