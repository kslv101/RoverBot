#pragma once
#include <cstdint>
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>
#include <iostream>

#include "Triggers.h"
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
    mathLib::Vec2 initialPosition{ -1.0f, -1.0f };
    mathLib::Vec2 currentPosition{ -1.0f, -1.0f };
    float yaw{ 0.0f };

    // Ошибки оборудования
    Triggers triggers;

    // Карта и навигация
    std::string mapPath = "maps/warehouse";
    bool mapLoaded = false;
    NavigationMap map;

    // Путь
    std::vector<mathLib::Vec2> currentPath;
    bool hasPlannedPath = false;
    int currentWaypointIndex = -1;

    // Цель
    int selectedTargetId = -1;  // обычный int, не atomic
    Target selectedTarget;

    // runtime resources
    std::vector<uint8_t> imageBuffer; // пример буфера для кадра
    std::mutex mtx;                   // для защиты не-атомарных полей
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

inline bool allocateBuffers(Robot& r, const InitParams& p)
{
    try
    {
        r.imageBuffer.clear();
        r.imageBuffer.resize(p.imageBufferBytes);
        return true;
    }
    catch (const std::bad_alloc&)
    {
        return false;
    }
}
