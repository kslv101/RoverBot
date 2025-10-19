#include "State.h"

std::string toString(State state)
{
    switch (state)
    {
    case State::Init:             return "Init";
    case State::Idle:             return "Idle";
    case State::Checkup:          return "Checkup";
    case State::PlanningRoute:    return "PlanningRoute";
    case State::MovingToTarget:   return "MovingToTarget";
    case State::AvoidingObstacle: return "AvoidingObstacle";
    case State::ApproachingTarget:return "ApproachingTarget";
    case State::EmergencyStop:    return "EmergencyStop";
    case State::Wait:             return "Wait";
    case State::RealSense:        return "RealSense";
    case State::ReadControl:      return "ReadControl";
    case State::SendControl:      return "SendControl";
    default:                      return "UnknownState";
    }
}