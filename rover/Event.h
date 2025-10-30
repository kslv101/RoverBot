// Event.h
#pragma once
#include <cstdint>
#include <optional>

enum class EventType : std::uint8_t
{
    None = 0,

    // ������� �� ���������
    TargetSelected,
    StartMission,
    GlobalStop,

    // �� �������� / ���������
    ObstacleDetected,
    TargetInView,
    PathBlocked,

    // �� ������������
    PathPlanningSucceeded,
    PathPlanningFailed,

    // �� ����������� ����
    WaypointReached,
    DestinationReached,

    // ������
    ArduinoError,
    RealSenseError,
    InternalError
};