#pragma once
#include <cstdint>
#include <string>

// ��������� ��������� ������
enum class State : std::uint8_t
{
    Init,
    Idle,

    Planning,
    ExecutingPath,
    Docking,

    EmergencyStop
};

std::string toString(State s);
