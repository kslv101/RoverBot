#include "NavigationMap.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"


bool NavigationMap::loadFromFile(const std::string& mapBasePath)
{
    const std::string yamlPath = mapBasePath + ".yaml";
    const std::string pgmPath = mapBasePath + ".png";
    const std::string targetsPath = mapBasePath + "_targets.yaml";

    if (!loadYamlMetadata(yamlPath)) return false;
    if (!loadPngImage(pgmPath)) return false;
    if (!loadTargetsYaml(targetsPath)) return false;

    log(LogLevel::Info, "Map loaded successfully: " + mapBasePath);
    return true;
}

bool NavigationMap::loadYamlMetadata(const std::string& yamlPath)
{
    try
    {
        YAML::Node node = YAML::LoadFile(yamlPath);
        grid.resolution = node["resolution"].as<float>(0.05f);
        grid.origin.x = node["origin"][0].as<float>(0.0f);
        grid.origin.y = node["origin"][1].as<float>(0.0f);

        // Размеры пока неизвестны — узнаем из PGM
        return true;
    }
    catch (const std::exception& e)
    {
        log(LogLevel::Error, "Failed to parse map YAML: " + std::string(e.what()));
        return false;
    }
}

bool NavigationMap::loadPngImage(const std::string& pngPath)
{
    int width, height, channels;
    unsigned char* data = stbi_load(pngPath.c_str(), &width, &height, &channels, 1);
    if (!data)
    {
        log(LogLevel::Error, "Failed to load PNG: " + pngPath);
        return false;
    }

    grid.width = width;
    grid.height = height;
    grid.data.resize(width * height);

    // Загружаем с инверсией Y → grid в мировых координатах
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            // data: [0][0] = верхний левый
            // grid: [0][0] = нижний левый → y_grid = height - 1 - y
            uint8_t pixel = data[y * width + x];
            int gridY = height - 1 - y;
            grid.data[gridY * width + x] = pixel;
        }
    }

    stbi_image_free(data);

    // Конвертация значений
    for (auto& pixel : grid.data)
    {
        if (pixel < 100) pixel = 100;     // чёрное → occupied
        else if (pixel > 200) pixel = 0;  // белое → free
        else pixel = 255;                 // серое → unknown
    }

    saveGridAsDebugPng(grid, "debug_grid.png");
    return true;
}

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

void NavigationMap::saveGridAsDebugPng(const OccupancyGrid& grid, const std::string& outputPath)
{
    if (grid.data.empty())
    {
        log(LogLevel::Error, "Cannot save debug PNG: grid is empty");
        return;
    }

    std::vector<uint8_t> pixels(grid.width * grid.height * 3);
    for (int y = 0; y < grid.height; ++y)
    {
        for (int x = 0; x < grid.width; ++x)
        {
            uint8_t val = grid.data[y * grid.width + x];
            uint8_t color = (val == 0) ? 255 : (val == 100) ? 0 : 128;

            // НЕ инвертируем Y — grid в мировых координатах,
            // но PNG требует верхний левый → инвертируем при записи
            int pngY = grid.height - 1 - y; // ← инверсия здесь
            int idx = (pngY * grid.width + x) * 3;
            pixels[idx] = pixels[idx + 1] = pixels[idx + 2] = color;
        }
    }

    if (stbi_write_png(outputPath.c_str(), grid.width, grid.height, 3, pixels.data(), 0))
    {
        log(LogLevel::Info, "Debug grid saved to: " + outputPath);
    }
    else
    {
        log(LogLevel::Error, "Failed to save debug PNG: " + outputPath);
    }
}

bool NavigationMap::loadTargetsYaml(const std::string& targetsPath)
{
    try
    {
        YAML::Node config = YAML::LoadFile(targetsPath);
        if (!config["targets"])
        {
            log(LogLevel::Error, "Targets YAML missing 'targets' key");
            return false;
        }

        targets.clear();
        for (const auto& node : config["targets"])
        {
            Target t;
            t.id = node["id"].as<int>();
            t.name = node["name"].as<std::string>();
            t.type = parseTargetType(node["type"].as<std::string>());
            t.position.x = node["pose"]["x"].as<float>();
            t.position.y = node["pose"]["y"].as<float>();
            targets.push_back(std::move(t));
        }
        return true;
    }
    catch (const std::exception& e)
    {
        log(LogLevel::Error, "Failed to load targets: " + std::string(e.what()));
        return false;
    }
}
