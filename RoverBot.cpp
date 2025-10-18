#include <functional>
#include <map>
#include <iostream>

#include "RoverBot.h"
#include "StateMachine.h"

int main()
{
    Robot cart; // Создание экземпляра объекта
    
    State currentState = State::TargetSelection;

    std::map<State, StateFunc> states; // Словарь состояний и функций переходов
    
    states[State::TargetSelection] = selectTarget;
    states[State::Planning] = plan;
    states[State::Moving] = move;
    states[State::Wait] = wait;
    states[State::RealSense] = realSense;
    states[State::ReadUpr] = readUpr;
    states[State::SendUpr] = sendUpr;

    // Цикл перехода из состояния в состояние
    while (cart.globalStop)
    {
        currentState = trigger(currentState, cart);
        if (auto it = states.find(currentState); it != states.end())
        {
            currentState = it->second(cart);
        }
        else
        {
            cart.globalStop = false;
        }
    }

    return 0;
}