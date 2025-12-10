#pragma once
#include <cstdint>
#include <string>

// Возможные состояния робота
enum class State : std::uint8_t
{
    Init,// Инициализация
    Idle,// Ожидание команд

    Planning,// Планирование маршрута
    ExecutingPath,// Движение по пути
    Docking,// Точный подъезд к цели

    EmergencyStop
};

std::string toString(State s);
