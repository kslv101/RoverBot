#pragma once
#include "State.h"
#include <functional>
#include <map>

// Forward declarations
struct Robot;
class EventQueue;
class MissionController;

class StateHandlers
{
public:
    explicit StateHandlers(Robot& robot, EventQueue& eventQueue, MissionController& missionController);

    StateHandlers(const StateHandlers&) = delete;
    StateHandlers& operator=(const StateHandlers&) = delete;

    using StateHandler = std::function<State(Robot&, EventQueue&)>;
    StateHandler getHandlerFor(State state) const;

private:
    // Поля класса
    Robot& robot;
    EventQueue& eventQueue;
    MissionController& missionController;
    std::map<State, StateHandler> handlerMap;

    State handleInit() const;
    State handleIdle() const;
    State handlePlanning() const;
    State handleExecutingPath() const;
    State handleDocking() const;
    State handleEmergencyStop() const;
};