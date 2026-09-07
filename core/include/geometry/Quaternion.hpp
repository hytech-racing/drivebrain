#pragma once

#include <algorithm>
#include <cmath>

namespace core::geometry
{

struct Quaternion
{
    double w{1.0};
    double x{0.0};
    double y{0.0};
    double z{0.0};

    [[nodiscard]] double norm() const noexcept
    {
        return std::sqrt(w * w + x * x + y * y + z * z);
    }

    [[nodiscard]] Quaternion normalized() const noexcept
    {
        const double len = norm();
        if (len <= 1e-12)
        {
            return Quaternion{};
        }

        return Quaternion{w / len, x / len, y / len, z / len};
    }

    [[nodiscard]] Quaternion multiply(const Quaternion& q) const noexcept
    {
        double rw = w * q.w - x * q.x - y * q.y - z * q.z;
        double rx = w * q.x + x * q.w + y * q.z - z * q.y;
        double ry = w * q.y - x * q.z + y * q.w + z * q.x;
        double rz = w * q.z + x * q.y - y * q.x + z * q.w;

        return Quaternion{rw, rx, ry, rz}.normalized();
    }

    [[nodiscard]] Quaternion operator*(const Quaternion& other) const noexcept
    {
        return multiply(other);
    }

    [[nodiscard]] Quaternion conjugate() const noexcept
    {
        return Quaternion{w, -x, -y, -z};
    }

    [[nodiscard]] Quaternion inverse() const noexcept
    {
        return normalized().conjugate();
    }

    [[nodiscard]] static Quaternion slerp(const Quaternion& start,
                                          const Quaternion& end,
                                          const double alpha) noexcept
    {
        Quaternion q0 = start.normalized();
        Quaternion q1 = end.normalized();

        double dot = q0.w * q1.w + q0.x * q1.x + q0.y * q1.y + q0.z * q1.z;
        if (dot < 0.0)
        {
            q1 = Quaternion{-q1.w, -q1.x, -q1.y, -q1.z};
            dot = -dot;
        }

        dot = std::clamp(dot, -1.0, 1.0);

        if (dot > 0.9995)
        {
            return Quaternion{q0.w + alpha * (q1.w - q0.w),
                              q0.x + alpha * (q1.x - q0.x),
                              q0.y + alpha * (q1.y - q0.y),
                              q0.z + alpha * (q1.z - q0.z)}
                .normalized();
        }

        const double theta_0 = std::acos(dot);
        const double theta = theta_0 * alpha;
        const double sin_theta = std::sin(theta);
        const double sin_theta_0 = std::sin(theta_0);

        const double s0 = std::cos(theta) - dot * sin_theta / sin_theta_0;
        const double s1 = sin_theta / sin_theta_0;

        return Quaternion{s0 * q0.w + s1 * q1.w, s0 * q0.x + s1 * q1.x,
                          s0 * q0.y + s1 * q1.y, s0 * q0.z + s1 * q1.z}
            .normalized();
    }
};

}  // namespace core::geometry
