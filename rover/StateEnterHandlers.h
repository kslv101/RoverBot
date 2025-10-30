// StateEnterHandlers.h
#pragma once
#include <functional>
#include "State.h"
#include "Robot.h"
#include "EventQueue.h"

using EnterHandler = std::function<void(Robot&, EventQueue&)>;

void onEnterPlanning(Robot& rover, EventQueue& events);
void onEnterExecutingPath(Robot& rover, EventQueue& events);
void onEnterEmergencyStop(Robot& rover, EventQueue& events);


extern const std::map<State, EnterHandler> STATE_ENTER_HANDLERS;