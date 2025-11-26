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
#include <PathPlanner.h>

// Глобальный флаг для корректного завершения
static std::atomic<bool> g_interruptRequested{ false };

extern "C" void signalHandler(int) noexcept
{
    g_interruptRequested.store(true, std::memory_order_relaxed);
}

int main()
{
    // Регистрация обработчика сигнала
    std::signal(SIGINT, signalHandler);
#ifdef SIGTERM
    std::signal(SIGTERM, signalHandler);
#endif

    EventQueue eventQueue;
    Robot rover;
    rover.mapPath = "maps/warehouse";
    if (!rover.map.loadFromFile(rover.mapPath))
    {
        log(LogLevel::Error, "Map loading failed!");
        return -1;
    }
    rover.mapLoaded = true;
    rover.currentPosition = { 0.5f, 0.5f };

    rover.selectedTargetId = 0;
    rover.selectedTarget = rover.map.getTargets()[0];
    eventQueue.push(EventType::TargetSelected);
    eventQueue.push(EventType::StartMission);

    startCommandListener(rover, eventQueue);

    State currentState = State::Idle;
    log(LogLevel::State, "Starting FSM in state: " + toString(currentState));

    State previousState = currentState;

    using namespace std::chrono_literals;

    // Цикл перехода из состояния в состояние
    while (!g_interruptRequested.load(std::memory_order_relaxed))
    {
        auto fsmIt = STATE_HANDLERS.find(currentState);
        if (fsmIt == STATE_HANDLERS.end())
        {
            log(LogLevel::Error, "Unknown state!");
            break;
        }
        State newState = fsmIt->second(rover, eventQueue);

        if (newState != currentState)
        {
            std::cout << "  |\n  v\n";
            log(LogLevel::State, toString(newState));

            // Выполняем побочные эффекты при входе в новое состояние
            auto enterIt = STATE_ENTER_HANDLERS.find(newState);
            if (enterIt != STATE_ENTER_HANDLERS.end())
            {
                enterIt->second(rover, eventQueue);
            }

            currentState = newState;
        }

        std::this_thread::sleep_for(10ms); // Небольшая пауза, чтобы не грузить CPU
    }

    stopCommandListener();
    log(LogLevel::Info, "RoverBot shutdown complete.");
    return 0;
}