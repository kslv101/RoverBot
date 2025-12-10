#include "StateMachine.h"
#include "MissionController.h"
#include "Logger.h"

StateMachine::StateMachine(Robot& robot, EventQueue& eventQueue, MissionController& missionController)
    : m_robot(robot)
    , m_eventQueue(eventQueue)
    , m_stateHandlers(robot, eventQueue, missionController)
    , m_stateEnterHandlers(robot, eventQueue, missionController)
{}

void StateMachine::run()
{
    log(LogLevel::Info, "StateMachine: starting main loop");

    while (m_running.load())
    {
        // Получаем обработчик для текущего состояния
        StateHandlers::StateHandler handler = m_stateHandlers.getHandlerFor(m_currentState);

        if (!handler)
        {
            log(LogLevel::Error, "StateMachine: no handler for state " +
                std::to_string(static_cast<int>(m_currentState.load())));
            m_running = false;
            break;
        }

        // Вызываем обработчик
        State newState = handler(m_robot, m_eventQueue);

        // Если состояние изменилось
        if (newState != m_currentState)
        {
            log(LogLevel::Info, "StateMachine: transition " +
                std::to_string(static_cast<int>(m_currentState.load())) + " -> " +
                std::to_string(static_cast<int>(newState)));

            // Вызываем onEnter для нового состояния
            StateEnterHandlers::EnterHandler enterHandler = m_stateEnterHandlers.getHandlerFor(newState);
            if (enterHandler)
            {
                enterHandler(m_robot, m_eventQueue);
            }

            // Обновляем текущее состояние
            m_currentState = newState;
        }

        // Задержка для предотвращения перегрузки CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    log(LogLevel::Info, "StateMachine: main loop stopped");
}

void StateMachine::stop()
{
    log(LogLevel::Info, "StateMachine: stop requested");
    m_running = false;
}