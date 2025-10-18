#include <functional>
#include <map>
#include <iostream>

#include "RoverBot.h"

// Возможные состояния объекта
enum class State : std::uint8_t
{
    TargetSelection,
    Planning,
    Moving,
    Wait,
    RealSense,
    ReadUpr,
    SendUpr
};

struct Triggers
{
    bool arduinoError{};
    bool realSenseError{};
    bool correctionError{};
};

struct Robot
{
    // Координаты
    int x{};
    int y{};

    // Переходы состояний
    bool start{};
    bool planningImpossible{};
    bool movingOk{};
    bool findObstacle{};
    bool findTarget{};
    bool missionComplete{};

    Triggers triggers;

    bool globalStop{ true };
};

// Функция обработки триггеров
State trigger(State state, Robot const& cart)
{
    if (cart.triggers.arduinoError ||
        cart.triggers.realSenseError ||
        cart.triggers.correctionError)
    {
        return State::TargetSelection;
    }
    return state;
}

// Функции состояний
State selectTarget(Robot& cart)
{
    if (cart.start)
    {
        return State::Planning;
    }
    else
    {
        return State::TargetSelection;
    }
};

State plan(Robot& cart)
{
    if (cart.planningImpossible)
    {
        return State::TargetSelection;
    }
    else
    {
        return State::Moving;
    }
};

State move(Robot& cart)
{
    return State::Wait;
}

State wait(Robot& cart)
{
    if (cart.movingOk)
    {
        return State::RealSense;
    }
    else
    {
        return State::Wait;
    }
};

State realSense(Robot& cart)
{
    if (cart.findObstacle)
    {
        return State::Planning;
    }
    else if (cart.findTarget)
    {
        return State::Planning;
    }
    else
    {
        return State::Planning;
    }
};

State readUpr(Robot& cart)
{
    if (cart.missionComplete)
    {
        return State::TargetSelection;
    }
    else
    {
        return State::SendUpr;
    }
};

State sendUpr(Robot& cart)
{
    return State::ReadUpr;
}

// Тип функций для подачи их в словарь
using StateFunc = std::function<State(Robot&)>;

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