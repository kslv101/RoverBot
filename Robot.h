#pragma once
#include <cstdint>
#include <atomic>
#include <thread>
#include <chrono>
#include <mutex>

#include <string_view>

#include "Triggers.h"
#include "Target.h"
#include "Math.h"
#include <iostream>

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
    math::Vec2  position{ 0.0f , 0.0f };
    float yaw{ 0.0f };

    Target target;

    // Переходы состояний
    std::atomic<bool> start{ false };            // событие: одноразовое (consume)
    std::atomic<bool> planningImpossible{ false }; // событие (consume)
    std::atomic<bool> movingOk{ false };         // событие (consume)
    std::atomic<bool> findObstacle{ false };     // уровень/событие по выбору (consume)
    std::atomic<bool> findTarget{ false };       // событие (consume)
    std::atomic<bool> missionComplete{ false };  // событие (consume)

    Triggers triggers;

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

//DEBUG
#include <source_location>

template <auto V>
inline constexpr std::string_view enum_name() noexcept
{
    std::string_view sv = std::source_location::current().function_name();
    auto first = sv.find_last_of(':');
    auto last = sv.find_last_of('>');
    return (first != sv.npos && last != sv.npos)
        ? sv.substr(first + 1, last - first - 1)
        : "Unknown";
}

template <typename E>
inline std::string_view toString(E e) noexcept
{
    switch (e)
    {
        case E::Init:       return enum_name<E::Init>();
        case E::Idle:       return enum_name<E::Idle>();
        case E::Checkup:    return enum_name<E::Checkup>();
        case E::PlanningRoute:   return enum_name<E::PlanningRoute>();
        case E::Moving:     return enum_name<E::Moving>();
        case E::Wait:       return enum_name<E::Wait>();
        case E::RealSense:  return enum_name<E::RealSense>();
        case E::ReadUpr:    return enum_name<E::ReadUpr>();
        case E::SendUpr:    return enum_name<E::SendUpr>();
    }
    return "Unknown";
}