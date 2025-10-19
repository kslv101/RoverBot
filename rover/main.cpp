#include <functional>
#include <map>
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>  // для обработки Ctrl+C

#include "StateMachine.h"
#include "Logger.h"
#include "NetworkCommand.h"

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

    constexpr bool isDebugEnabled = true;

    int i = 0;

    Robot rover; // Создание экземпляра объекта
    startCommandListener(rover); // ← ЗАПУСК СЕТЕВОГО ПОТОКА
    

    const std::map<State, StateFunc> stateMachine = { // Словарь состояний и функций переходов
        { State::Init, init },
        { State::Idle, idle },
        { State::PlanningRoute, plan },
        { State::MovingToTarget, move },
        { State::AvoidingObstacle, avoidObstacle }, // ← добавьте эту функцию
        { State::ApproachingTarget, approachTarget },// ← добавьте
        { State::Wait, wait },
    };

    State currentState = State::Init;
    State previousState = currentState;


    if constexpr (isDebugEnabled)
    {
        log(LogLevel::State, toString(currentState));
    }
    
    using namespace std::chrono_literals;

    // Цикл перехода из состояния в состояние
    while (!rover.globalStop && !g_interruptRequested.load(std::memory_order_relaxed))
    {
        // 1. Проверяем внешние триггеры (асинхронные события)
        State triggeredState = trigger(currentState, rover);
        if (triggeredState != currentState)
        {
            currentState = triggeredState;
        }
        else
        {
            // 2. Выполняем логику текущего состояния
            auto it = stateMachine.find(currentState);
            if (it != stateMachine.end())
            {
                currentState = it->second(rover);
            }
            else
            {
                log(LogLevel::Error, "Encountered unknown state! Stopping.");
                rover.globalStop = true;
                break;
            }
        }

        // Логируем переход состояний
        if (currentState != previousState)
        {
            previousState = currentState;
            if constexpr (isDebugEnabled)
            {
                std::cout << "  |\n  v\n";
                log(LogLevel::State, toString(currentState));
            }
        }

        
        std::this_thread::sleep_for(10ms); // Небольшая пауза, чтобы не грузить CPU
    }

    stopCommandListener();
    log(LogLevel::Info, "RoverBot shutdown complete.");
    return 0;
}