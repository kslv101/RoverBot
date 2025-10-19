#pragma once
#include "Robot.h"
#include <thread>
#include <atomic>

void startCommandListener(Robot& rover);// Запускает фоновый поток для приёма UDP-команд

void stopCommandListener(); // Корректно останавливает поток