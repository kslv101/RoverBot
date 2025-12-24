#pragma once
#include "Robot.h"
#include "EventQueue.h"

struct RoutePoint 
{
    float x;
    float y;
};

// Конфигурация интерфейса 
constexpr const char* GUI_IP = "10.55.250.87";
constexpr int GUI_PORT = 8888;

// Функции для работы с маршрутом
void sendRouteToGUI(const std::vector<RoutePoint>& keypoints,const mathLib::Vec2& start, const mathLib::Vec2& goal, const std::string& routeId = "");

void startCommandListener(Robot& rover, EventQueue& eventQueue);
void stopCommandListener();