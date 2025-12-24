#pragma once
#include <cstdint>
#include <optional>
#include <variant>
#include <vector>
#include "MathLib.h"

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
    PathPlanningResult,

    // От исполнителя пути
    WaypointReached,
    DestinationReached,

    // Ошибки
    ArduinoError,
    RealSenseError,
    InternalError,

    ResetEmergency
};

struct ObstacleData
{
    mathLib::Vec2 position; // Позиция препятствия в мировых координатах
    float distance;         // Расстояние до препятствия [m]

    ObstacleData() = default;
    ObstacleData(mathLib::Vec2 pos, float dist) : position(pos), distance(dist) {}
};

struct PlanningResultData
{
    bool success;                    // true = путь найден, false = ошибка
    std::vector<mathLib::Vec2> path; // Вектор пути (пустой если failed)

    PlanningResultData() = default;
    PlanningResultData(bool succ, std::vector<mathLib::Vec2> p)
        : success(succ), path(std::move(p))
    {}
};

// Универсальное событие
struct Event
{
    EventType type;

    // Вариант: может быть либо пустым, либо данными
    std::variant<
        std::monostate,           // для событий без данных
        ObstacleData,             // для ObstacleDetected
        PlanningResultData        // для PathPlanningResult
    > data;

    // Конструктор для событий без данных
    explicit Event(EventType eventType) : type(eventType), data(std::monostate{}) {}
    // Конструктор для событий с данными
    Event(EventType eventType, ObstacleData obstacleData) : type(eventType), data(std::move(obstacleData)) {}
    Event(EventType eventType, PlanningResultData planningData) : type(eventType), data(std::move(planningData)) {}

    static Event obstacleDetected(mathLib::Vec2 position, float distance)
    {
        return Event(EventType::ObstacleDetected, ObstacleData{ position, distance });
    }

    static Event pathPlanned(std::vector<mathLib::Vec2> path)
    {
        return Event(EventType::PathPlanningResult, PlanningResultData{ true, std::move(path) });
    }

    static Event planningFailed()
    {
        return Event(EventType::PathPlanningResult, PlanningResultData{ false, {} });
    }

    // Проверка, содержит ли событие данные указанного типа
    template <typename T>
    bool holdsData() const
    {
        return std::holds_alternative<T>(data);
    }

    // Безопасное получение данных (проверяет тип)
    template <typename T>
    const T* getData() const
    {
        return std::get_if<T>(&data);
    }

    // Безопасное получение данных с проверкой типа
    template <typename T>
    std::optional<T> tryGetData() const
    {
        if (holdsData<T>())
        {
            return std::get<T>(data);
        }
        return std::nullopt;
    }
};