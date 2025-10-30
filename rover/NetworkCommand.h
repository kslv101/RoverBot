#pragma once
#include "Robot.h"
#include "EventQueue.h"

void startCommandListener(Robot& rover, EventQueue& eventQueue);
void stopCommandListener();