#pragma once
#include <cstdint>

#include "MathLib.h"

enum class TargetType : std::uint8_t
{
    Circle,
    Square,
    Triangle
};

struct Target
{
    int id;
    std::string name;
    TargetType type{TargetType::Circle};
    mathLib::Vec2 position{0.0f,0.0f};
};



inline TargetType parseTargetType(const std::string& s)
{
    if (s == "Circle" || s == "circle") return TargetType::Circle;
    if (s == "Square" || s == "square") return TargetType::Square;
    if (s == "Triangle" || s == "triangle") return TargetType::Triangle;
    throw std::runtime_error("Unknown target type: " + s);
}

inline std::string targetTypeToString(TargetType t)
{
    switch (t)
    {
        case TargetType::Circle: return "Circle";
        case TargetType::Square: return "Square";
        case TargetType::Triangle: return "Triangle";
    }
    return "Unknown";
}