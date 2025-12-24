#pragma once
#include "State.h"
#include <functional>
#include <map>

struct Robot;
class EventQueue;
class MissionController;

class StateEnterHandlers
{
public:
    explicit StateEnterHandlers(Robot& robot, EventQueue& eventQueue, MissionController& missionController);

    using EnterHandler = std::function<void(Robot&, EventQueue&)>;

    // Возвращаем обработчик для конкретного состояния
    EnterHandler getHandlerFor(State state) const;

private:
    Robot& m_robot;
    EventQueue& m_eventQueue;
    MissionController& m_missionController;

    // Приватные методы
    void onEnterIdle() const;
    void onEnterPlanning() const;
    void onEnterExecutingPath() const;
    void onEnterEmergencyStop() const;

    std::map<State, EnterHandler> m_enterMap;
};

