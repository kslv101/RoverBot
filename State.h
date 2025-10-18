#include <cstdint>
// Возможные состояния робота
enum class State : std::uint8_t
{
    Init, //инициализация массивов, ожидание команды оператора
    Idle, //ожидание
    Checkup, //проверка
    PlanningRoute,//планирование маршрута
    Moving,
    LastMileMoving,
    Wait, //ожидание
    RealSense,
    ReadUpr,
    SendUpr
};