#pragma once
#include <cstdint>
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>
#include <iostream>

#include "Target.h"
#include "MathLib.h"
#include "NavigationMap.h"

using namespace std::chrono_literals;

struct InitParams
{
    int serialRetries = 3;
    int cameraRetries = 3;
    std::chrono::milliseconds retryDelay{ 200 };
    size_t imageBufferBytes = 1280 * 720 * 3; 
    std::chrono::seconds maxInitDuration{ 10 };
};

struct Robot
{
    InitParams params;
    // Поза
    mutable std::mutex poseMtx;
    
    mathLib::Vec2 initialPosition{ -1.0f, -1.0f };// Начальная позиция (устанавливается один раз)
    mathLib::Vec2 currentPosition{ -1.0f, -1.0f };// Текущая позиция (обновляется одометрией)
    float yaw{ 0.0f }; // Угол поворота

    // Вспомогательная структура для атомарного снапшота
    struct PoseSnapshot
    {
        mathLib::Vec2 position;
        float yaw;
    };

    PoseSnapshot getPose() const
    {
        std::lock_guard<std::mutex> lock(poseMtx);
        return { currentPosition, yaw };
    }

    void setPose(const mathLib::Vec2& pos, float newYaw)
    {
        std::lock_guard<std::mutex> lock(poseMtx);
        currentPosition = pos;
        yaw = newYaw;
    }

    // для одометрии
    void updatePose(float deltaX, float deltaY, float deltaYaw)
    {
        std::lock_guard<std::mutex> lock(poseMtx);
        currentPosition.x += deltaX;
        currentPosition.y += deltaY;
        yaw += deltaYaw;
    }

    // Карта и навигация
    mutable std::mutex mapMtx;
    std::string mapPath = "maps/warehouse";
    std::atomic<bool> mapLoaded{ false };
    NavigationMap map;

    std::shared_ptr<const OccupancyGrid> getGrid() const
    {
        return map.getGrid();
    }


    // Путь
    mutable std::mutex pathMtx;

    std::vector<mathLib::Vec2> currentPath;
    bool hasPlannedPath = false;
    int currentWaypointIndex = -1;

    void setPath(std::vector<mathLib::Vec2>&& path)
    {
        std::lock_guard<std::mutex> lock(pathMtx);
        currentPath = std::move(path);
        hasPlannedPath = !currentPath.empty();
        currentWaypointIndex = hasPlannedPath ? 0 : -1;
        log(LogLevel::Info, "RobotState: Path updated with " +
            std::to_string(currentPath.size()) + " waypoints");
    }

    std::optional<mathLib::Vec2> getNextWaypoint() const
    {
        std::lock_guard<std::mutex> lock(pathMtx);
        if (!hasPlannedPath || currentWaypointIndex < 0 ||
            currentWaypointIndex >= static_cast<int>(currentPath.size()))
        {
            return std::nullopt;
        }
        return currentPath[currentWaypointIndex];
    }

    void advanceWaypoint()
    {
        std::lock_guard<std::mutex> lock(pathMtx);
        if (hasPlannedPath && currentWaypointIndex >= 0)
        {
            ++currentWaypointIndex;
            if (currentWaypointIndex >= static_cast<int>(currentPath.size()))
            {
                hasPlannedPath = false;
                currentWaypointIndex = -1;
            }
        }
    }

    void clearPath()
    {
        std::lock_guard<std::mutex> lock(pathMtx);
        currentPath.clear();
        hasPlannedPath = false;
        currentWaypointIndex = -1;
    }

    bool isDestinationReached() const
    {
        std::lock_guard<std::mutex> lock(pathMtx);
        return !hasPlannedPath && currentWaypointIndex == -1;
    }

    // Цель
    mutable std::mutex targetMtx;
    std::atomic<int> selectedTargetId{ -1 };
    Target selectedTarget;

    void setTarget(int id, const Target& t)
    {
        std::lock_guard<std::mutex> lock(targetMtx);
        selectedTarget = t;
        selectedTargetId.store(id);
        log(LogLevel::Info, "Target set: " + t.name + " (ID: " + std::to_string(id) + ")");
    }

    std::optional<Target> getTarget() const
    {
        std::lock_guard<std::mutex> lock(targetMtx);
        int id = selectedTargetId.load();
        if (id < 0) return std::nullopt;
        return selectedTarget;
    }

    int getTargetId() const
    {
        return selectedTargetId.load(); // atomic
    }

    void clearTarget()
    {
        std::lock_guard<std::mutex> lock(targetMtx);
        selectedTargetId.store(-1);
        selectedTarget = Target{}; // сбрасываем
    }
};


inline bool checkSerialDevice(std::string port, int baud, std::string& out_version)
{
    std::this_thread::sleep_for(50ms);
    out_version = "??";
    return true;
}

inline bool checkCameraDevice(int camIndex)
{
    std::this_thread::sleep_for(50ms);
    return true;
}

inline bool checkRealSense()
{
    std::this_thread::sleep_for(30ms);
    return true;
}

//inline bool allocateBuffers(Robot& r, const InitParams& p)
//{
//    try
//    {
//        r.imageBuffer.clear();
//        r.imageBuffer.resize(p.imageBufferBytes);
//        return true;
//    }
//    catch (const std::bad_alloc&)
//    {
//        return false;
//    }
//}
