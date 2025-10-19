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

using namespace std::chrono_literals;

struct InitParams
{
    int serialRetries = 3;
    int cameraRetries = 3;
    std::chrono::milliseconds retryDelay{ 200 };
    size_t imageBufferBytes = 1280 * 720 * 3; 
    size_t mapBytes = 1024 * 1024;
    std::chrono::seconds maxInitDuration{ 10 };
};

struct Robot
{
    InitParams params;
    mathLib::Vec2  position{ 0.0f , 0.0f };
    float yaw{ 0.0f };

    Target target;

    // Переходы состояний
    //std::atomic<bool> start{ false };            // событие: одноразовое (consume)
    std::atomic<bool> planningImpossible{ false }; // событие (consume)
    std::atomic<bool> movingOk{ false };         // событие (consume)
    std::atomic<bool> findObstacle{ false };     // уровень/событие по выбору (consume)
    std::atomic<bool> findTarget{ false };       // событие (consume)
    std::atomic<bool> missionComplete{ false };  // событие (consume)

    Triggers triggers;

    // Сетевые команды
    std::atomic<bool> newCommandReceived{ false };
    std::atomic<int> pendingTargetId{ -1 }; // ID цели из карты
    std::atomic<bool> start{ false };
    std::atomic<bool> globalStop{ false };

    // runtime resources
    std::vector<uint8_t> imageBuffer; // пример буфера для кадра
    std::vector<uint8_t> mapBuffer;   // occupancy grid memory
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
        r.mapBuffer.clear();
        r.mapBuffer.resize(p.mapBytes);
        return true;
    }
    catch (const std::bad_alloc&)
    {
        return false;
    }
}

inline void logInfo(const std::string& s) { std::cout << "[INFO] " << s << std::endl; }
inline void logWarn(const std::string& s) { std::cout << "[WARN] " << s << std::endl; }
inline void logError(const std::string& s) { std::cerr << "[ERROR] " << s << std::endl; }
