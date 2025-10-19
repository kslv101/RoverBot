// PathPlanner.cpp
#include "PathPlanner.h"
#include "Logger.h"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>

namespace pathplanner
{

    // Вспомогательные функции
    mathLib::IVec2 worldToGrid(const mathLib::Vec2& world, const OccupancyGrid& grid)
    {
        int x = static_cast<int>((world.x - grid.origin.x) / grid.resolution);
        int y = static_cast<int>((world.y - grid.origin.y) / grid.resolution);
        return { x, y };
    }

    mathLib::Vec2 gridToWorld(const mathLib::IVec2& cell, const OccupancyGrid& grid)
    {
        float x = grid.origin.x + (cell.x + 0.5f) * grid.resolution; // центр ячейки
        float y = grid.origin.y + (cell.y + 0.5f) * grid.resolution;
        return { x, y };
    }

    bool isCellWalkable(int x, int y, const OccupancyGrid& grid)
    {
        if (x < 0 || x >= grid.width || y < 0 || y >= grid.height)
            return false;
        uint8_t val = grid.data[y * grid.width + x];
        return (val < 65); // ROS: < 65 = free (occupied_thresh = 0.65 → 0.65*100=65)
    }

    float heuristic(const mathLib::IVec2& a, const mathLib::IVec2& b)
    {
        // Евклидово расстояние
        int dx = a.x - b.x;
        int dy = a.y - b.y;
        return static_cast<float>(std::sqrt(dx * dx + dy * dy));
    }

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

    PathResult computePathAStar(
        const OccupancyGrid& grid,
        const mathLib::Vec2& startInMeters,
        const mathLib::Vec2& goalInMeters
    )
    {
        PathResult result;

        // Преобразуем в ячейки
        mathLib::IVec2 startCell = worldToGrid(startInMeters, grid);
        mathLib::IVec2 goalCell = worldToGrid(goalInMeters, grid);

        // Проверка границ
        if (!isCellWalkable(startCell.x, startCell.y, grid))
        {
            log(LogLevel::Warn, "A*: Start cell is not walkable");
            return result;
        }
        if (!isCellWalkable(goalCell.x, goalCell.y, grid))
        {
            log(LogLevel::Warn, "A*: Goal cell is not walkable");
            return result;
        }

        // Если старт == цель
        if (startCell.x == goalCell.x && startCell.y == goalCell.y)
        {
            result.success = true;
            result.pathInMeters.push_back(goalInMeters);
            return result;
        }

        // Open set: min-heap по f
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openSet;
        openSet.push({ startCell, 0.0f, heuristic(startCell, goalCell) });

        // Closed set и g-scores
        std::unordered_map<mathLib::IVec2, float, std::hash<mathLib::IVec2>> gScore;
        std::unordered_map<mathLib::IVec2, mathLib::IVec2, std::hash<mathLib::IVec2>> cameFrom;

        gScore[startCell] = 0.0f;

        // Направления (4-связность)
        const mathLib::IVec2 directions[4] = {
            { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 }
        };

        while (!openSet.empty())
        {
            Node current = openSet.top();
            openSet.pop();

            // Если достигли цели
            if (current.pos.x == goalCell.x && current.pos.y == goalCell.y)
            {
                // Восстанавливаем путь
                std::vector<mathLib::IVec2> pathCells;
                mathLib::IVec2 step = current.pos;
                while (!(step.x == startCell.x && step.y == startCell.y))
                {
                    pathCells.push_back(step);
                    step = cameFrom[step];
                }
                pathCells.push_back(startCell);
                std::reverse(pathCells.begin(), pathCells.end());

                // Конвертируем в мировые координаты
                for (const auto& cell : pathCells)
                {
                    result.pathInMeters.push_back(gridToWorld(cell, grid));
                }
                result.success = true;
                return result;
            }

            // Проверяем соседей
            for (const auto& dir : directions)
            {
                mathLib::IVec2 neighbor = { current.pos.x + dir.x, current.pos.y + dir.y };

                if (!isCellWalkable(neighbor.x, neighbor.y, grid))
                    continue;

                float tentativeG = gScore[current.pos] + 1.0f; // шаг = 1 ячейка

                if (gScore.find(neighbor) == gScore.end() || tentativeG < gScore[neighbor])
                {
                    cameFrom[neighbor] = current.pos;
                    gScore[neighbor] = tentativeG;
                    float f = tentativeG + heuristic(neighbor, goalCell);
                    openSet.push({ neighbor, tentativeG, f });
                }
            }
        }

        log(LogLevel::Warn, "A*: No path found");
        return result;
    }

} // namespace pathplanner