#pragma once
#include "Robot.h"
#include <functional>

using StateFunc = std::function<State(Robot&)>;

State trigger(State state, const Robot& cart);

State selectTarget(Robot& cart);
State plan(Robot& cart);
State move(Robot& cart);
State wait(Robot& cart);
State realSense(Robot& cart);
State readUpr(Robot& cart);
State sendUpr(Robot& cart);