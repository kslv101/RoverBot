#pragma once
#include <vector>
#include <atomic>
#include <memory>
#include "MathLib.h"
#include "NavigationMap.h"

namespace pathplanner
{
    // Параметры планирования
    struct PlanningParams
    {
        std::shared_ptr<const OccupancyGrid> grid;
        mathLib::Vec2 startInMeters;
        mathLib::Vec2 goalInMeters;
    };

    struct Node
    {
        mathLib::IVec2 pos;
        float g = 0.0f; // cost from start
        float f = 0.0f; // g + h

        bool operator>(const Node& other) const
        {
            return f > other.f; // для std::priority_queue (min-heap)
        }
    };

    // Результат планирования
    struct PathResult
    {
        bool succeeded = false;
        std::vector<mathLib::Vec2> pathInMeters;

        PathResult() = default;
        PathResult(bool succ, std::vector<mathLib::Vec2> path)
            : succeeded(succ), pathInMeters(std::move(path))
        {}

        static PathResult success(std::vector<mathLib::Vec2> path)
        {
            return PathResult(true, std::move(path));
        }

        static PathResult failure()
        {
            return PathResult(false, {});
        }

        explicit operator bool() const { return succeeded; }
    };

    class PathPlanner
    {
    public:
        PathPlanner();
        ~PathPlanner() = default;

        // Удаляем копирование
        PathPlanner(const PathPlanner&) = delete;
        PathPlanner& operator=(const PathPlanner&) = delete;

        // Управление прерыванием
        void interrupt() { m_interruptFlag = true; }
        void clearInterrupt() { m_interruptFlag = false; }
        bool isInterrupted() const { return m_interruptFlag.load(); }

        // Основной метод
        PathResult computePathAStar(const PlanningParams& params);
        // Оптимизация до ключевых точек
        std::vector<mathLib::Vec2> extractKeypoints(const std::vector<mathLib::Vec2>& rawPath, float minAngleDeg = 30.0f, float maxDistance = 1.0f) const;

    private:
        mathLib::IVec2 worldToGrid(const mathLib::Vec2& world) const;
        mathLib::Vec2 gridToWorld(const mathLib::IVec2& cell) const;
        bool isCellWalkable(int x, int y) const;
        bool isPositionValid(const mathLib::Vec2& pos) const;
        float heuristic(const mathLib::IVec2& a, const mathLib::IVec2& b) const;
        bool shouldInterrupt(int iteration) const;

        static constexpr uint8_t WALKABLE_THRESHOLD = 64; // val < WALKABLE_THRESHOLD -> walkable
        std::atomic<bool> m_interruptFlag{ false };
        std::shared_ptr<const OccupancyGrid> m_currentGrid; // Устанавливается в computePathAStar
    };
} // namespace pathplanner