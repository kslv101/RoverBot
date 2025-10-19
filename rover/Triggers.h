#pragma once
#include <atomic>

struct Triggers
{
    std::atomic<bool> internalError{ false };    // ���� �� �������� �������
    std::atomic<bool> arduinoError{ false };    // ���� �� �������� �������
    std::atomic<bool> realSenseError{ false };   // ���� �� �������� �������
    std::atomic<bool> correctionError{ false };  // ���� �� �������� �������
};