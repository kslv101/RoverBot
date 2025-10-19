#pragma once
#include <vector>
#include <string>
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
    bool loadFromFile(const std::string& mapBasePath); // без расширения

    const OccupancyGrid& getGrid() const { return grid; }
    const std::vector<Target>& getTargets() const { return targets; }
    bool isValid() const { return !grid.data.empty() && !targets.empty(); }

private:
    OccupancyGrid grid;
    std::vector<Target> targets;

    bool loadYamlMetadata(const std::string& yamlPath);
    bool loadPngImage(const std::string& pgmPath);
    void saveGridAsDebugPng(const OccupancyGrid& grid, const std::string& outputPath);
    bool loadTargetsYaml(const std::string& targetsPath);
};