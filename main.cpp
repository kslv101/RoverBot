#include <functional>
#include <map>
#include <iostream>
#include <thread>
#include <chrono>

#include "StateMachine.h"

#ifdef _WIN32
#  include <windows.h>
#endif

void setColor(int code)          // 2 = зелёный, 4 = красный, 7 = белый
{
#ifdef _WIN32
    static const HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(h, code);
#else
    static const char* seq [] = { "\033[0m", "\033[32m", "\033[31m", "\033[37m" };
    std::cout << seq[code];
#endif
}

int main()
{
    bool isDebugEnabled = true;
    int i = 0;

    Robot rover; // Создание экземпляра объекта
    
    State currentState = State::Init;
    State previousState = currentState;

    std::map<State, StateFunc> states; // Словарь состояний и функций переходов
    
    states[State::Init] = init;
    states[State::Idle] = idle;
    states[State::PlanningRoute] = plan;
    states[State::Moving] = move;
    states[State::Wait] = wait;
    states[State::RealSense] = realSense;
    states[State::ReadUpr] = readUpr;
    states[State::SendUpr] = sendUpr;

    if (isDebugEnabled)
    {
        // ---- рисуем текущее состояние ----
        setColor(2);   // зелёный
        std::cout << "[STATE] " << toString(currentState) << '\n';
        setColor(7);   // белый
        
    }
    
    // Цикл перехода из состояния в состояние
    while (!rover.globalStop)
    {
        i++;
        currentState = trigger(currentState, rover);
        if (auto it = states.find(currentState); it != states.end())
        {
            currentState = it->second(rover);
        }
        else
        {
            rover.globalStop = false;
        }
        //if (i==100)
        //{
        //    currentState = State::Moving;
        //}

        if (currentState != previousState)
        {
            previousState = currentState;

            if (isDebugEnabled)
            {
                setColor(7);   // белый
                std::cout << "  |\n"
                    "  |\n"
                    "  v\n";

                setColor(2);   // зелёный

                std::cout << "[STATE] " << toString(currentState) << '\n';
                setColor(7);   // белый

                std::this_thread::sleep_for(std::chrono::milliseconds(300));
            }
        }
    }

    return 0;
}