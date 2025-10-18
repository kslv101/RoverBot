#pragma once
#include <cstdint>
#include "Triggers.h"

// Возможные состояния объекта
enum class State : std::uint8_t
{
    TargetSelection,
    Planning,
    Moving,
    Wait,
    RealSense,
    ReadUpr,
    SendUpr
};

struct Robot
{
    // Координаты
    int x{};
    int y{};

    // Переходы состояний
    bool start{};
    bool planningImpossible{};
    bool movingOk{};
    bool findObstacle{};
    bool findTarget{};
    bool missionComplete{};

    Triggers triggers;

    bool globalStop{ true };
};