#pragma once
#include "Robot.h"
#include <functional>
#include "State.h"
#include "Logger.h"

using StateFunc = std::function<State(Robot&)>;

State trigger(State state, Robot& rover);

State init(Robot& rover);
State idle(Robot& rover);

State plan(Robot& rover);
State move(Robot& rover);
State avoidObstacle(Robot& rover);
State approachTarget(Robot& rover);
State wait(Robot& rover);
State realSense(Robot& rover);
State readControl(Robot& rover);
State sendControl(Robot& rover);