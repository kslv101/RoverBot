// Event.h
#pragma once
#include <cstdint>
#include <optional>

enum class EventType : std::uint8_t
{
    None = 0,

    // Команды от оператора
    TargetSelected,
    StartMission,
    GlobalStop,

    // От датчиков / навигации
    ObstacleDetected,
    TargetInView,
    PathBlocked,

    // От планировщика
    PathPlanningSucceeded,
    PathPlanningFailed,

    // От исполнителя пути
    WaypointReached,
    DestinationReached,

    // Ошибки
    ArduinoError,
    RealSenseError,
    InternalError
};