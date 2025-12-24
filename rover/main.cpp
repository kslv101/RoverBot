#include <iostream>
#include <thread>
#include <chrono>
#include <functional>
#include <map>
#include <csignal>  // для обработки Ctrl+C
#include <atomic>
#include <vector>

#include "StateHandlers.h"  
#include "StateEnterHandlers.h"
#include "Logger.h"
#include "NetworkCommand.h"
#include "EventQueue.h"

#include "Robot.h"
#include "PathPlanner.h"
#include "MissionController.h"
#include "UartDriver.h"
#include "StateMachine.h"

// Глобальный флаг для корректного завершения
static std::atomic<bool> g_interruptRequested{ false };

extern "C" void signalHandler(int) noexcept
{
    g_interruptRequested.store(true, std::memory_order_relaxed);
}

int main()
{
    Robot robot;
    EventQueue eventQueue;
    UartDriver uart("/dev/ttyUSB1", 115200);
    uart.connect();

    MissionController missionController(robot, uart);
    startCommandListener(robot, eventQueue);

    robot.mapPath = "maps/warehouse";
    if (!robot.map.loadFromFile(robot.mapPath))
    {
        log(LogLevel::Error, "Map loading failed!");
        return -1;
    }
    robot.mapLoaded.store(true);

    StateMachine stateMachine(robot, eventQueue, missionController);
    try
    {
        eventQueue.push(Event(EventType::StartMission));
        stateMachine.run();
    }
    catch (const std::exception& e)
    {
        log(LogLevel::Error, "Main loop crashed: " + std::string(e.what()));
        return -1;
    }

    stateMachine.stop();
    uart.disconnect();

    return 0;
}