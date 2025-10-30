#include "State.h"

std::string toString(State state)
{
    switch (state)
    {
    case State::Init: return "Init";
    case State::Idle: return "Idle";

    case State::Planning: return "Planning";
    case State::ExecutingPath: return "ExecutingPath";
    case State::Docking: return "Docking";

    case State::EmergencyStop: return "EmergencyStop";

    default: return "UnknownState";
    }
}