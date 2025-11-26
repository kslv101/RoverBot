// PathPlanner.cpp
#include "PathPlanner.h"
#include "Logger.h"
#include <queue>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

namespace pathplanner
{
    PathPlanner::PathPlanner() : m_interruptFlag(false) {}

    // Вспомогательные функции
    inline int indexFromCell(int x, int y, int width) { return y * width + x; }

    mathLib::IVec2 PathPlanner::worldToGrid(const mathLib::Vec2& world) const
    {
        int x = static_cast<int>(std::floor((world.x - m_currentGrid->origin.x) / m_currentGrid->resolution));
        int y = static_cast<int>(std::floor((world.y - m_currentGrid->origin.y) / m_currentGrid->resolution));
        return { x, y };
    }

    mathLib::Vec2 PathPlanner::gridToWorld(const mathLib::IVec2& cell) const
    {
        float x = m_currentGrid->origin.x + (cell.x + 0.5f) * m_currentGrid->resolution;
        float y = m_currentGrid->origin.y + (cell.y + 0.5f) * m_currentGrid->resolution;
        return { x, y };
    }

    bool PathPlanner::isCellWalkable(int x, int y) const
    {
        if (x < 0 || x >= m_currentGrid->width || y < 0 || y >= m_currentGrid->height)
            return false;
        uint8_t val = m_currentGrid->data[y * m_currentGrid->width + x];
        return (val <= WALKABLE_THRESHOLD);
    }

    bool PathPlanner::isPositionValid(const mathLib::Vec2& pos) const
    {
        mathLib::IVec2 cell = worldToGrid(pos);
        return isCellWalkable(cell.x, cell.y);
    }

    float PathPlanner::heuristic(const mathLib::IVec2& a, const mathLib::IVec2& b) const
    {
        int dx = a.x - b.x;
        int dy = a.y - b.y;
        return static_cast<float>(std::sqrt(static_cast<double>(dx * dx + dy * dy)));
    }

    bool PathPlanner::shouldInterrupt(int iteration) const
    {
        if (iteration % 100 == 0 && m_interruptFlag.load(std::memory_order_relaxed))
        {
            log(LogLevel::Info, "A*: interrupted by request");
            return true;
        }
        return false;
    }

    // compressGridPath: сжимает путь из ячеек в минимальные точки смены направления
    static std::vector<mathLib::IVec2> compressGridPath(const std::vector<mathLib::IVec2>& cells)
    {
        std::vector<mathLib::IVec2> out;
        if (cells.empty()) return out;

        // Если путь длиной 1 — сразу вернуть
        if (cells.size() == 1)
        {
            out.push_back(cells.front());
            return out;
        }

        out.reserve(cells.size());
        out.push_back(cells.front());

        // Начальная дискретная дирекция между 0 и 1
        auto sign = [](int v) -> int { return (v > 0) - (v < 0); };

        int prev_sdx = sign(cells[1].x - cells[0].x);
        int prev_sdy = sign(cells[1].y - cells[0].y);

        // идём от третьей вершины
        for (size_t i = 2; i < cells.size(); ++i)
        {
            int sdx = sign(cells[i].x - cells[i - 1].x);
            int sdy = sign(cells[i].y - cells[i - 1].y);

            // если направление поменялось — фиксируем конец предыдущего run (cells[i-1])
            if (sdx != prev_sdx || sdy != prev_sdy)
            {
                out.push_back(cells[i - 1]);
                prev_sdx = sdx;
                prev_sdy = sdy;
            }
            // иначе продолжаем run
        }

        // всегда добавляем последнюю клетку
        out.push_back(cells.back());
        return out;
    }

    pathplanner::PathResult PathPlanner::computePathAStar(const PlanningParams& params)
    {
        m_currentGrid = params.grid;
        if (!m_currentGrid)
        {
            log(LogLevel::Error, "A*: Grid is null!");
            return PathResult::failure();
        }

        m_interruptFlag = false;// cбрасываем флаг перед началом планирования

        // Проверка границ
        if (!isPositionValid(params.startInMeters))
        {
            log(LogLevel::Warn, "A*: Start position is not walkable");
            return PathResult::failure();
        }
        if (!isPositionValid(params.goalInMeters))
        {
            log(LogLevel::Warn, "A*: Goal position is not walkable");
            return PathResult::failure();
        }

        // Преобразуем в ячейки
        mathLib::IVec2 startCell = worldToGrid(params.startInMeters);
        mathLib::IVec2 goalCell = worldToGrid(params.goalInMeters);

        // Если старт == цель
        if (startCell.x == goalCell.x && startCell.y == goalCell.y)
        {
            return PathResult::success({ params.goalInMeters });
        }

        const int width = m_currentGrid->width;
        const int height = m_currentGrid->height;
        const int total = width * height;

        // Используем линейные массивы для производительности
        const float INF = std::numeric_limits<float>::infinity();
        std::vector<float> gScore(total, INF);         // лучшая найденная стоимость до ячейки
        std::vector<int> cameFrom(total, -1);          // индекс предка в виде single int (y*width + x)
        std::vector<uint8_t> closed(total, 0);         // закрытый набор

        auto idx = [&](const mathLib::IVec2& c) -> int { return c.y * width + c.x; };
        auto idxXY = [&](int x, int y) -> int { return y * width + x; };

        // Приоритетная очередь: элементы с наименьшим f вверху
        struct PQNode { mathLib::IVec2 pos; float g; float f; };
        struct PQCmp { bool operator()(PQNode const& a, PQNode const& b) const { return a.f > b.f; } };
        std::priority_queue<PQNode, std::vector<PQNode>, PQCmp> open;

        // Инициализация
        gScore[idx(startCell)] = 0.0f;
        open.push({ startCell, 0.0f, heuristic(startCell, goalCell) });

        // 8 направлений (dx,dy)
        const mathLib::IVec2 directions[8] = {
            { 1, 0 }, { -1, 0 }, { 0, 1 }, { 0, -1 },
            { 1, 1 }, { 1, -1 }, { -1, 1 }, { -1, -1 }
        };

        int iteration = 0;
        while (!open.empty())
        {
            if (shouldInterrupt(++iteration))
            {
                return PathResult::failure();
            }

            PQNode current = open.top();
            open.pop();

            // Проверка индекса и границ (дополнительная защита)
            if (current.pos.x < 0 || current.pos.x >= width || current.pos.y < 0 || current.pos.y >= height)
                continue;

            int curIndex = idx(current.pos);

            // Пропуск уже закрытого узла
            if (closed[curIndex]) continue;

            // Пропуск устаревшего элемента: если g у current больше чем актуальный gScore
            if (current.g > gScore[curIndex] + 1e-6f) continue;

            // Отмечаем как закрытый
            closed[curIndex] = 1;

            // Проверка достижения цели
            if (current.pos.x == goalCell.x && current.pos.y == goalCell.y)
            {
                // Восстановление пути через cameFrom (индексы)
                std::vector<mathLib::IVec2> pathCells;
                int stepIndex = curIndex;
                while (!(stepIndex == idx(startCell)))
                {
                    int sx = stepIndex % width;
                    int sy = stepIndex / width;
                    pathCells.push_back({ sx, sy });
                    int from = cameFrom[stepIndex];
                    if (from < 0)
                    {
                        // cameFrom сломан, возвращаем ошибку
                        log(LogLevel::Error, "A*: cameFrom broken during path reconstruction");
                        return PathResult::failure();
                    }
                    stepIndex = from;
                }
                // добавляем старт
                pathCells.push_back(startCell);
                std::reverse(pathCells.begin(), pathCells.end());

                // Сжимаем путь по дискретным направлениям
                auto compressedCells = compressGridPath(pathCells);

                std::vector<mathLib::Vec2> path;
                path.reserve(compressedCells.size());
                for (const auto& cell : compressedCells)
                {
                    path.push_back(gridToWorld(cell));
                }

                log(LogLevel::Info, "A*: Path found with " + std::to_string(path.size()) + " waypoints");
                return PathResult::success(std::move(path));
            }

            // Расширяем соседей
            for (const auto& dir : directions)
            {
                int nx = current.pos.x + dir.x;
                int ny = current.pos.y + dir.y;

                // Границы
                if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

                int nidx = idxXY(nx, ny);

                if (closed[nidx]) continue;
                if (!isCellWalkable(nx, ny)) continue;

                float stepCost = (dir.x == 0 || dir.y == 0) ? 1.0f : std::sqrt(2.0f);
                float tentativeG = gScore[curIndex] + stepCost;

                if (tentativeG + 1e-6f < gScore[nidx])
                {
                    gScore[nidx] = tentativeG;
                    cameFrom[nidx] = curIndex;
                    float f = tentativeG + heuristic({ nx, ny }, goalCell);
                    open.push({ { nx, ny }, tentativeG, f });
                }
            }
        }

        log(LogLevel::Warn, "A*: No path found");
        return PathResult::failure();
    }

    std::vector<mathLib::Vec2> PathPlanner::extractKeypoints(const std::vector<mathLib::Vec2>& rawPath, float /*minAngleDeg*/, float maxDistance) const
    {
        if (rawPath.size() < 3) return rawPath;

        std::vector<mathLib::Vec2> filtered;
        filtered.reserve(rawPath.size());

        const float minSegmentLength = maxDistance; // используем maxDistance как порог (в метрах)

        // Всегда сохраняем старт
        filtered.push_back(rawPath.front());

        for (size_t i = 1; i < rawPath.size() - 1; ++i)
        {
            const auto& candidate = rawPath[i];
            const auto& lastKept = filtered.back();

            float dist = std::hypot(candidate.x - lastKept.x, candidate.y - lastKept.y);
            if (dist >= minSegmentLength)
            {
                filtered.push_back(candidate);
            }
            else
            {
                // отбрасываем слишком близкую точку
            }
        }

        filtered.push_back(rawPath.back());

        log(LogLevel::Info, "Filtered path from " +
            std::to_string(rawPath.size()) + " to " +
            std::to_string(filtered.size()) + " points (minSeg=" + std::to_string(minSegmentLength) + "m)");
        return filtered;
    }

} // namespace pathplanner
