#pragma once
#include <atomic>

struct Triggers
{
    std::atomic<bool> internalError{ false };    // пока не очистили вручную
    std::atomic<bool> arduinoError{ false };    // пока не очистили вручную
    std::atomic<bool> realSenseError{ false };   // пока не очистили вручную
    std::atomic<bool> correctionError{ false };  // пока не очистили вручную
};