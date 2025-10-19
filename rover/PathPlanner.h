#pragma once
#include <vector>
#include "MathLib.h"
#include "NavigationMap.h"

namespace pathplanner
{

    struct PathResult
    {
        bool success = false;
        std::vector<mathLib::Vec2> pathInMeters;
    };

    PathResult computePathAStar(
        const OccupancyGrid& grid,
        const mathLib::Vec2& startInMeters,
        const mathLib::Vec2& goalInMeters
    );

} // namespace pathplanner