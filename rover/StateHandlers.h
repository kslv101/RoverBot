#pragma once
#include "State.h"

#include <optional>
#include <Robot.h>
#include <EventQueue.h>
#include <functional>

using StateHandler = std::function<State(const Robot&, EventQueue&)>;
extern const std::map<State, StateHandler> STATE_HANDLERS;
//
State handleInit(const Robot& rover, EventQueue& events);
State handleIdle(const Robot& rover, EventQueue& events);
State handlePlanning(const Robot& rover, EventQueue& events);
State handleExecutingPath(const Robot& rover, EventQueue& events);
State handleDocking(const Robot& rover, EventQueue& events);
State handleEmergencyStop(const Robot& rover, EventQueue& events);