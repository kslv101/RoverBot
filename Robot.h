#pragma once
#include <cstdint>
#include "Triggers.h"

// ��������� ��������� �������
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
    // ����������
    int x{};
    int y{};

    // �������� ���������
    bool start{};
    bool planningImpossible{};
    bool movingOk{};
    bool findObstacle{};
    bool findTarget{};
    bool missionComplete{};

    Triggers triggers;

    bool globalStop{ true };
};