#include "StateHandlers.h"
#include <PathPlanner.h>

const std::map<State, StateHandler> STATE_HANDLERS = {
    { State::Init, handleInit },
    { State::Idle, handleIdle },
    { State::Planning, handlePlanning },
    { State::ExecutingPath, handleExecutingPath },
    { State::Docking, handleDocking },
    { State::EmergencyStop, handleEmergencyStop }
};

State handleInit(const Robot& rover, EventQueue& events)
{
    while (auto event = events.pop())
    {
        if (*event == EventType::ArduinoError ||
            *event == EventType::RealSenseError ||
            *event == EventType::InternalError)
        {
            log(LogLevel::Warn, "Init: hardware error detected. Awaiting reset.");
            return State::Init;
        }
    }
    log(LogLevel::Info, "Initialization completed successfully. Entering Idle state.");

    return State::Idle;
}

State handleIdle(const Robot& rover, EventQueue& events)
{
    while (auto event = events.pop())
    {
        switch (*event)
        {
        case EventType::TargetSelected:
            log(LogLevel::Info, "Target selected. Awaiting Start command.");
            break;

        case EventType::StartMission:
            if (rover.selectedTargetId < 0)
            {
                log(LogLevel::Warn, "Start received, but no target selected!");
                return State::Idle;
            }
            log(LogLevel::Info, "Starting mission to target: " + rover.selectedTarget.name);
            return State::Planning;

        case EventType::GlobalStop:
            log(LogLevel::Info, "Global stop received (ignored in Idle).");
            break;

        case EventType::ArduinoError:
        case EventType::RealSenseError:
        case EventType::InternalError:
            log(LogLevel::Error, "Hardware error in Idle. Entering EmergencyStop.");
            return State::EmergencyStop;

        default:
            break;
        }
    }
    return State::Idle;
}

State handlePlanning(const Robot& rover, EventQueue& events)
{
    // Проверки
    if (!rover.mapLoaded)
    {
        log(LogLevel::Error, "Planning: map not loaded!");
        return State::Idle;
    }
    if (rover.selectedTargetId < 0)
    {
        log(LogLevel::Error, "Planning: no target selected!");
        return State::Idle;
    }

    log(LogLevel::Info, "Computing path with A*...");

    auto result = pathplanner::computePathAStar(
        rover.map.getGrid(),
        rover.currentPosition,           // текущая позиция
        rover.selectedTarget.position    // цель в метрах
    );

    if (result.success && !result.pathInMeters.empty())
    {
        log(LogLevel::Info, "Path planned successfully with " +
            std::to_string(result.pathInMeters.size()) + " waypoints.");

        return State::ExecutingPath;
    }
    else
    {
        log(LogLevel::Warn, "Path planning failed.");
        return State::Idle;
    }
}

State handleExecutingPath(const Robot& rover, EventQueue& events)
{
    while (auto event = events.pop())
    {
        switch (*event)
        {
        case EventType::ObstacleDetected:
        case EventType::PathBlocked:
            log(LogLevel::Warn, "Obstacle detected. Stopping motion.");
            // Можно перейти в Replanning, но пока в Idle
            return State::Idle;

        case EventType::TargetInView:
            log(LogLevel::Info, "Target in view. Switching to docking.");
            return State::Docking;

        case EventType::DestinationReached:
            log(LogLevel::Info, "Destination reached. Mission complete.");
            return State::Idle;

        case EventType::GlobalStop:
            log(LogLevel::Info, "Global stop during motion.");
            return State::Idle;

        default:
            break;
        }
    }

    
    return State::ExecutingPath; // Пока просто остаёмся в состоянии
}

State handleDocking(const Robot& rover, EventQueue& events)
{
    while (auto event = events.pop())
    {
        if (*event == EventType::DestinationReached)
        {
            log(LogLevel::Info, "Docking complete.");
            return State::Idle;
        }
        if (*event == EventType::GlobalStop)
        {
            return State::Idle;
        }
    }
    return State::Docking;
}

State handleEmergencyStop(const Robot& rover, EventQueue& events)
{
    while (auto event = events.pop())
    {
        if (*event == EventType::GlobalStop)
        {
            // остаёмся в стопе
        }
    }
    // Остаёмся в EmergencyStop до ручного сброса 
    return State::EmergencyStop;
}