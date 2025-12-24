#pragma once
#include <cmath>
#include <cstddef> // для std::size_t
#include <functional>

namespace mathLib
{
    class Vec2
    {
    public:
        float x{}, y{};
        constexpr Vec2() = default;
        constexpr Vec2(float x_, float y_) : x(x_), y(y_) {}
    };

    class IVec2
    {
    public:
        int x{}, y{};
        constexpr IVec2() = default;
        constexpr IVec2(int x_, int y_) : x(x_), y(y_) {}
    };

    class Vec3
    {
    public:
        float x{}, y{}, z{};
        constexpr Vec3() = default;
        constexpr Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
    };

    constexpr float dot(const Vec2& a, const Vec2& b) noexcept
    {
        return a.x * b.x + a.y * b.y;
    }

    constexpr float dot(const Vec3& a, const Vec3& b) noexcept
    {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    constexpr Vec2 operator-(const Vec2& a, const Vec2& b) noexcept
    {
        return Vec2(a.x - b.x, a.y - b.y);
    }

    constexpr Vec3 operator-(const Vec3& a, const Vec3& b) noexcept
    {
        return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
    }

    constexpr bool operator==(const Vec2& a, const Vec2& b) noexcept
    {
        return a.x == b.x && a.y == b.y;
    }

    constexpr bool operator<(const Vec2& a, const Vec2& b) noexcept
    {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    }

    constexpr bool operator==(const IVec2& a, const IVec2& b) noexcept
    {
        return a.x == b.x && a.y == b.y;
    }

    constexpr bool operator<(const IVec2& a, const IVec2& b) noexcept
    {
        return a.x < b.x || (a.x == b.x && a.y < b.y);
    }

    float magnitude(const Vec2& v) noexcept;
    float magnitude(const Vec3& v) noexcept;
    float angleBetween(const Vec2& a, const Vec2& b) noexcept;
    float angleBetween(const Vec3& a, const Vec3& b) noexcept;
} // namespace mathLib

namespace std
{
    template<>
    struct hash<mathLib::IVec2>
    {
        size_t operator()(const mathLib::IVec2& v) const noexcept
        {
            return hash<int>()(v.x) ^ (hash<int>()(v.y) << 1);
        }
    };
}