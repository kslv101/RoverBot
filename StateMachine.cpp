#include "StateMachine.h"

State trigger(State state, const Robot& cart)
{
    if (cart.triggers.arduinoError ||
        cart.triggers.realSenseError ||
        cart.triggers.correctionError)
    {
        return State::TargetSelection;
    }
    return state;
}

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