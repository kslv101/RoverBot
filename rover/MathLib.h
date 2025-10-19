#pragma once
#include <cmath>

namespace mathLib
{
    class Vec2
    {
    public:
        float x{}, y{};
        constexpr Vec2() = default;
        constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}
    };

    class Vec3
    {
    public:
        float x{}, y{}, z{};
        constexpr Vec3() = default;
        constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    };

    /* ---------- свободные функции вместо статических методов ---------- */
    constexpr float  dot(const Vec2& a, const Vec2& b) noexcept;
    constexpr float  dot(const Vec3& a, const Vec3& b) noexcept;
    float            magnitude(const Vec2& v) noexcept;
    float            magnitude(const Vec3& v) noexcept;
    float            angleBetween(const Vec2& a, const Vec2& b) noexcept;
    float            angleBetween(const Vec3& a, const Vec3& b) noexcept;
    constexpr Vec2   operator-(const Vec2& a, const Vec2& b) noexcept;
    constexpr Vec3   operator-(const Vec3& a, const Vec3& b) noexcept;
} // namespace math