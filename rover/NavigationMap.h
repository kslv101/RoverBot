#pragma once
#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <stdexcept>

#include "MathLib.h"
#include "Target.h"
#include "Logger.h"

struct OccupancyGrid
{
    std::vector<uint8_t> data; // 0 = free, 100 = occupied, 255 = unknown (как в ROS)
    int width = 0;
    int height = 0;
    float resolution = 0.05f; // meters per cell
    mathLib::Vec2 origin{ 0.0f, 0.0f }; // world coords of (0,0) cell
};

class NavigationMap
{
public:
    NavigationMap() = default;

    bool loadFromFile(const std::string& mapBasePath);

    std::shared_ptr<const OccupancyGrid> getGrid() const
    {
        std::lock_guard<std::mutex> lock(gridMutex);
        return grid; // копируем указатель (атомарно)
    }
    const std::vector<Target>& getTargets() const { return targets; }
    bool isValid() const
    {
        std::lock_guard<std::mutex> lock(gridMutex);
        return grid != nullptr && !grid->data.empty() && !targets.empty();
    }
    void inflateObstacles(float robotRadius);

private:
    mutable std::mutex gridMutex;
    std::shared_ptr<OccupancyGrid> grid;
    std::vector<Target> targets;

    static constexpr uint8_t OCCUPIED = 100;
    static constexpr uint8_t FREE = 0;
    static constexpr uint8_t UNKNOWN = 255;
   
    bool loadYamlMetadata(const std::string& yamlPath, OccupancyGrid& grid);
    bool loadPngImage(const std::string& pngPath, OccupancyGrid& grid);
    void saveGridAsDebugPng(const OccupancyGrid& grid, const std::string& outputPath) const;
    bool loadTargetsYaml(const std::string& targetsPath);
};