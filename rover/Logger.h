#pragma once
#include <iostream>
#include <string>

enum class LogLevel
{
    Info,
    Warn,
    Error,
    Debug,
    State
};

void log(LogLevel level, const std::string& message);