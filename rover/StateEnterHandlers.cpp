#include "StateEnterHandlers.h"
#include "PathPlanner.h"
#include "Logger.h"

void onEnterPlanning(Robot& rover, EventQueue& events)
{
    // Проверки
    if (!rover.mapLoaded)
    {
        log(LogLevel::Error, "Planning: map not loaded!");
        events.push(EventType::PathPlanningFailed);
    }
    if (rover.selectedTargetId < 0)
    {
        log(LogLevel::Error, "Planning: no target selected!");
        events.push(EventType::PathPlanningFailed);
    }
    log(LogLevel::Info, "Entering Planning state: computing A* path...");

    auto result = pathplanner::computePathAStar(
        rover.map.getGrid(),
        rover.currentPosition,
        rover.selectedTarget.position
    );

    if (result.success && !result.pathInMeters.empty())
    {
        rover.currentPath = std::move(result.pathInMeters);
        rover.hasPlannedPath = true;
        rover.currentWaypointIndex = 0;

        log(LogLevel::Info, "Path planned successfully with " +
            std::to_string(rover.currentPath.size()) + " waypoints.");

        events.push(EventType::PathPlanningSucceeded);
    }
    else
    {
        events.push(EventType::PathPlanningFailed);
    }
}

void onEnterExecutingPath(Robot& rover, EventQueue& events)
{
    log(LogLevel::Info, "Entering ExecutingPath: starting motion controller");

}

void onEnterEmergencyStop(Robot& rover, EventQueue& events)
{
    log(LogLevel::Error, "Entering EmergencyStop: halting all systems");

}

// Состояния, где ничего не нужно делать при входе
void onEnterIdle(Robot&, EventQueue&) {}
void onEnterDocking(Robot&, EventQueue&) {}
void onEnterInit(Robot&, EventQueue&) {}

const std::map<State, EnterHandler> STATE_ENTER_HANDLERS = {
    { State::Init, onEnterInit },
    { State::Idle, onEnterIdle },
    { State::Planning, onEnterPlanning },
    { State::ExecutingPath, onEnterExecutingPath },
    { State::Docking, onEnterDocking },
    { State::EmergencyStop, onEnterEmergencyStop }
};