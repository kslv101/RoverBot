// StateEnterHandlers.cpp
#include "StateEnterHandlers.h"
#include "Logger.h"
#include "Event.h"
#include "EventQueue.h"
#include "PathPlanner.h"
#include "MissionController.h"
#include "Robot.h"
#include <NetworkCommand.h>

StateEnterHandlers::StateEnterHandlers(Robot& robot, EventQueue& eventQueue, MissionController& missionController)
    : m_robot(robot), m_eventQueue(eventQueue), m_missionController(missionController)
{
    m_enterMap = {
        { State::Init, [](Robot&, EventQueue&) {} },
        { State::Idle, [this](Robot&, EventQueue&) { this->onEnterIdle(); } },
        { State::Planning, [this](Robot&, EventQueue&) { this->onEnterPlanning(); } },
        { State::ExecutingPath, [this](Robot&, EventQueue&) { this->onEnterExecutingPath(); } },
        { State::Docking, [](Robot&, EventQueue&) {} },
        { State::EmergencyStop, [this](Robot&, EventQueue&) { this->onEnterEmergencyStop(); } }
    };
}

StateEnterHandlers::EnterHandler StateEnterHandlers::getHandlerFor(State state) const
{
    auto it = m_enterMap.find(state);
    return (it != m_enterMap.end()) ? it->second : [](Robot&, EventQueue&) {};
}

void StateEnterHandlers::onEnterIdle() const
{
    log(LogLevel::Info, "Enter Idle state");

    // m_missionController.stopMission();
}

void StateEnterHandlers::onEnterPlanning() const
{
    log(LogLevel::Info, "Enter Planning state");

    // Проверки
    int targetId = m_robot.getTargetId();
    if (targetId < 0)
    {
        log(LogLevel::Error, "Planning: no target selected!");
        m_eventQueue.push(Event::planningFailed());
        return;
    }

    auto grid = m_robot.map.getGrid();
    if (!grid)
    {
        log(LogLevel::Error, "Planning: grid is null!");
        m_eventQueue.push(Event::planningFailed());
        return;
    }

    pathplanner::PlanningParams params;
    params.grid = grid;
    params.startInMeters = m_robot.getPose().position;

    auto target = m_robot.getTarget();
    if (!target.has_value())
    {
        log(LogLevel::Error, "Planning: target not available!");
        m_eventQueue.push(Event::planningFailed());
        return;
    }
    params.goalInMeters = target->position;

    // Сохраняем позицию цели для отправки в поток
    mathLib::Vec2 goalPosition = target->position;
    mathLib::Vec2 startPosition = params.startInMeters;

    // Запускаем планировщик в фоне
    std::thread([this, params, startPosition, goalPosition]()
                {
                    try
                    {
                        pathplanner::PathPlanner planner;
                        planner.clearInterrupt();
                        auto result = planner.computePathAStar(params);

                        if (result.succeeded && !result.pathInMeters.empty())
                        {
                            auto keypoints = planner.extractKeypoints(result.pathInMeters, 15.0f, 0.1f);

                            // Формируем данные для отправки на GUI
                            std::vector<RoutePoint> guiPoints;
                            for (const auto& wp : keypoints) {
                                guiPoints.push_back({ wp.x, wp.y });
                            }

                            // Отправляем маршрут на интерфейс
                            sendRouteToGUI(
                                guiPoints,
                                startPosition,
                                goalPosition,
                                "mission_" + std::to_string(std::time(nullptr))
                            );

                            m_missionController.startMission(keypoints);
                            m_eventQueue.push(Event::pathPlanned(result.pathInMeters));
                        }
                        else
                        {
                            m_eventQueue.push(Event::planningFailed());
                        }
                    }
                    catch (...)
                    {
                        m_eventQueue.push(Event::planningFailed());
                    }
                }).detach();
}

void StateEnterHandlers::onEnterExecutingPath() const
{
    log(LogLevel::Info, "Entering ExecutingPath: starting motion controller");
}

void StateEnterHandlers::onEnterEmergencyStop() const
{
    log(LogLevel::Error, "Entering EmergencyStop: halting all systems");
}

//#include "StateEnterHandlers.h"
//#include "PathPlanner.h"
//#include "Logger.h"
//#include <StateHandlers.h>
//using namespace pathplanner;
//
//void onEnterPlanning(Robot& rover, EventQueue& events)
//{
//    // Проверки
//    int targetId = rover.selectedTargetId.load(std::memory_order_acquire);
//    if (targetId < 0)
//    {
//        log(LogLevel::Error, "Planning: no target selected!");
//        events.push(Event::planningFailed());
//        return;
//    }
//    auto grid = rover.map.getGrid();
//    if (!grid)
//    {
//        log(LogLevel::Error, "Planning: grid is null!");
//        events.push(Event::planningFailed());
//        return;
//    }
//    PlanningParams params;
//    params.grid = grid;
//    params.startInMeters = rover.getPose().position;
//
//    auto targetOpt = rover.getTarget(); // блокирует targetMtx
//    if (!targetOpt.has_value())
//    {
//        log(LogLevel::Error, "Planning: target not available!");
//        events.push(Event::planningFailed());
//        return;
//    }
//    params.goalInMeters = targetOpt->position;
//
//    log(LogLevel::Info, "Entering Planning: launching A* in background");
//
//    std::thread([params, &events]()
//                {
//                    try
//                    {
//                        PathPlanner planner;
//                        planner.clearInterrupt();
//                        auto result = planner.computePathAStar(params);
//                        if (result.succeeded && !result.pathInMeters.empty())
//                        {
//                            // Оптимизируем путь
//                            auto keypoints = planner.extractKeypoints(result.pathInMeters);
//
//                            // Передаём в mission controller
//                            if (g_missionController)
//                            {
//                                g_missionController->startMission(keypoints);
//                            }
//
//                            events.push(Event::pathPlanned(std::move(result.pathInMeters)));
//                        }
//                        else
//                        {
//                            events.push(Event::planningFailed());
//                        }
//                    }
//                    catch (const std::exception& e)
//                    {
//                        log(LogLevel::Error, std::string("Planning exception: ") + e.what());
//                        events.push(Event::planningFailed());
//                    }
//                }).detach();
//}
//
//void onEnterExecutingPath(Robot& rover, EventQueue& events)
//{
//    log(LogLevel::Info, "Entering ExecutingPath: starting motion controller");
//
//}
//
//void onEnterEmergencyStop(Robot& rover, EventQueue& events)
//{
//    log(LogLevel::Error, "Entering EmergencyStop: halting all systems");
//
//}
//
//// Состояния, где ничего не нужно делать при входе
//void onEnterIdle(Robot&, EventQueue&) {}
//void onEnterDocking(Robot&, EventQueue&) {}
//void onEnterInit(Robot&, EventQueue&) {}
//
//const std::map<State, EnterHandler> STATE_ENTER_HANDLERS = {
//    { State::Init, onEnterInit },
//    { State::Idle, onEnterIdle },
//    { State::Planning, onEnterPlanning },
//    { State::ExecutingPath, onEnterExecutingPath },
//    { State::Docking, onEnterDocking },
//    { State::EmergencyStop, onEnterEmergencyStop }
//};