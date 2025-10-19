#include "MathLib.h"
#include <algorithm>
#include <cmath>

namespace mathLib
{
    float magnitude(const Vec2& v) noexcept
    {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }

    float magnitude(const Vec3& v) noexcept
    {
        return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
    }

    float angleBetween(const Vec2& a, const Vec2& b) noexcept
    {
        float denom = magnitude(a) * magnitude(b);
        if (denom == 0.0f) return 0.0f;
        float c = dot(a, b) / denom;
        c = std::clamp(c, -1.0f, 1.0f);
        return std::acos(c);
    }

    float angleBetween(const Vec3& a, const Vec3& b) noexcept
    {
        float denom = magnitude(a) * magnitude(b);
        if (denom == 0.0f) return 0.0f;
        float c = dot(a, b) / denom;
        c = std::clamp(c, -1.0f, 1.0f);
        return std::acos(c);
    }
} // namespace mathLib