#pragma once
#include <iostream>
#include <string>
#include <mutex>

enum class LogLevel
{
    Info,
    Warn,
    Error,
    Debug,
    State
};

extern std::mutex g_logMutex;

void log(LogLevel level, const std::string& message);