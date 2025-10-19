#include "MathLib.h"
#include <algorithm>

namespace mathLib
{
    constexpr float dot(const Vec2& a, const Vec2& b) noexcept
    {
        return a.x * b.x + a.y * b.y;
    }

    constexpr float dot(const Vec3& a, const Vec3& b) noexcept
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

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

    constexpr Vec2 operator-(const Vec2& a, const Vec2& b) noexcept
    {
        return Vec2(a.x - b.x, a.y - b.y);
    }

    constexpr Vec3 operator-(const Vec3& a, const Vec3& b) noexcept
    {
        return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
    }
} // namespace math