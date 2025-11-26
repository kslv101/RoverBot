// StateMachine.h
#pragma once
#include <atomic>
#include "State.h"
#include "StateHandlers.h"
#include "StateEnterHandlers.h"

class MissionController; // Forward declaration

class StateMachine
{
public:
    explicit StateMachine(Robot& robot, EventQueue& eventQueue, MissionController& missionController);

    // Запрещаем копирование
    StateMachine(const StateMachine&) = delete;
    StateMachine& operator=(const StateMachine&) = delete;
 
    void run();// Главный цикл FSM
    void stop();// Принудительная остановка

    State getCurrentState() const { return m_currentState.load(); }

private:
    std::atomic<State> m_currentState{ State::Init };
    std::atomic<bool> m_running{ true };

    // Обработчики
    StateHandlers m_stateHandlers;
    StateEnterHandlers m_stateEnterHandlers;

    Robot& m_robot;
    EventQueue& m_eventQueue;
};