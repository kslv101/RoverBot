#pragma once
#include "Robot.h"
#include <thread>
#include <atomic>

void startCommandListener(Robot& rover);// ��������� ������� ����� ��� ����� UDP-������

void stopCommandListener(); // ��������� ������������� �����