#pragma once
#include "Robot.h"
#include <functional>
#include "State.h"

using StateFunc = std::function<State(Robot&)>;

State trigger(State state, Robot& rover);

State init(Robot& rover);
State idle(Robot& rover);
State selectTarget(Robot& rover);
State plan(Robot& rover);
State move(Robot& rover);
State wait(Robot& rover);
State realSense(Robot& rover);
State readUpr(Robot& rover);
State sendUpr(Robot& rover);