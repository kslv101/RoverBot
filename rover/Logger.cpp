#include "Logger.h"
#ifdef _WIN32
#  include <windows.h>
#endif

void setConsoleColor(LogLevel level)
{
#ifdef _WIN32
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    switch (level)
    {
    case LogLevel::Info:   SetConsoleTextAttribute(h, 7); break;  // white
    case LogLevel::Warn:   SetConsoleTextAttribute(h, 6); break;  // yellow
    case LogLevel::Error:  SetConsoleTextAttribute(h, 4); break;  // red
    case LogLevel::Debug:  SetConsoleTextAttribute(h, 2); break;  // green
    case LogLevel::State:  SetConsoleTextAttribute(h, 3); break;  // cyan
    }
#else
    switch (level)
    {
    case LogLevel::Info:   std::cout << "\033[37m"; break; // white
    case LogLevel::Warn:   std::cout << "\033[33m"; break; // yellow
    case LogLevel::Error:  std::cout << "\033[31m"; break; // red
    case LogLevel::Debug:  std::cout << "\033[32m"; break; // green
    case LogLevel::State:  std::cout << "\033[36m"; break; // cyan
    }
#endif
}

void log(LogLevel level, const std::string& message)
{
    setConsoleColor(level);
    switch (level)
    {
    case LogLevel::Info:   std::cout << "[INFO] "; break;
    case LogLevel::Warn:   std::cout << "[WARN] "; break;
    case LogLevel::Error:  std::cerr << "[ERROR] "; break;
    case LogLevel::Debug:  std::cout << "[DEBUG] "; break;
    case LogLevel::State:  std::cout << "[STATE] "; break;
    }
    if (level == LogLevel::Error)
        std::cerr << message << "\033[0m" << std::endl;
    else
        std::cout << message << "\033[0m" << std::endl;
}