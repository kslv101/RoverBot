#include <cstdint>
#include "Math.h"

enum class TargetType : std::uint8_t
{
    Circle,
    Square,
    Triangle
};

struct Target
{
    TargetType type{TargetType::Circle};
    math::Vec2 position{0.0f,0.0f};
};