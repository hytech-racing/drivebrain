#pragma once
#include <algorithm>
#include <cmath>

#include "RigidTransform2D.hpp"

namespace transforms
{

struct Point3D
{
    double x_m{};
    double y_m{};
    double z_m{};
};

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

struct Pose3D
{
    double x_m{};
    double y_m{};
    double z_m{};
    Quaternion q{};

    [[nodiscard]] static Pose3D identity() noexcept { return Pose3D{}; }

    [[nodiscard]] static Pose3D from_xy_yaw(const double x_m,
                                            const double y_m,
                                            const double yaw_rad) noexcept
    {
        const double half_yaw = yaw_rad * 0.5;
        return Pose3D{x_m, y_m, 0.0,
                      Quaternion{std::cos(half_yaw), 0.0, 0.0,
                                 std::sin(half_yaw)}};
    }

    [[nodiscard]] static Pose3D from_pose2d(const Pose2D& pose) noexcept
    {
        return from_xy_yaw(pose.x_m, pose.y_m, pose.yaw_rad);
    }

    // Returns T^-1
    [[nodiscard]] Pose3D inverse() const noexcept
    {
        Quaternion inv_q = q.inverse();

        double num1 = inv_q.x * 2.0;
        double num2 = inv_q.y * 2.0;
        double num3 = inv_q.z * 2.0;
        double num4 = inv_q.x * num1;
        double num5 = inv_q.y * num2;
        double num6 = inv_q.z * num3;
        double num7 = inv_q.x * num2;
        double num8 = inv_q.x * num3;
        double num9 = inv_q.y * num3;
        double num10 = inv_q.w * num1;
        double num11 = inv_q.w * num2;
        double num12 = inv_q.w * num3;

        double inv_x = -((1.0 - (num5 + num6)) * x_m + (num7 - num12) * y_m +
                         (num8 + num11) * z_m);
        double inv_y = -((num7 + num12) * x_m + (1.0 - (num4 + num6)) * y_m +
                         (num9 - num10) * z_m);
        double inv_z = -((num8 - num11) * x_m + (num9 + num10) * y_m +
                         (1.0 - (num4 + num5)) * z_m);

        return Pose3D{inv_x, inv_y, inv_z, inv_q};
    }

    // Returns T_self * T_other
    [[nodiscard]] Pose3D operator*(const Pose3D& other) const noexcept
    {
        return compose(other);
    }

    // Returns T_self * T_other
    // Explicitly says "compose," functionally the same as the multiply operator
    [[nodiscard]] Pose3D compose(const Pose3D& other) const noexcept
    {
        const Quaternion normalized_q = q.normalized();

        double num1 = normalized_q.x * 2.0;
        double num2 = normalized_q.y * 2.0;
        double num3 = normalized_q.z * 2.0;
        double num4 = normalized_q.x * num1;
        double num5 = normalized_q.y * num2;
        double num6 = normalized_q.z * num3;
        double num7 = normalized_q.x * num2;
        double num8 = normalized_q.x * num3;
        double num9 = normalized_q.y * num3;
        double num10 = normalized_q.w * num1;
        double num11 = normalized_q.w * num2;
        double num12 = normalized_q.w * num3;

        double rot_x = (1.0 - (num5 + num6)) * other.x_m +
                       (num7 - num12) * other.y_m + (num8 + num11) * other.z_m;
        double rot_y = (num7 + num12) * other.x_m +
                       (1.0 - (num4 + num6)) * other.y_m +
                       (num9 - num10) * other.z_m;
        double rot_z = (num8 - num11) * other.x_m + (num9 + num10) * other.y_m +
                       (1.0 - (num4 + num5)) * other.z_m;

        double new_x = x_m + rot_x;
        double new_y = y_m + rot_y;
        double new_z = z_m + rot_z;

        Quaternion new_q = q.multiply(other.q);

        return Pose3D{new_x, new_y, new_z, new_q};
    }

    // Returns T_self * local_p
    // Allows you to write: Point3D global_p = my_transform * local_p
    [[nodiscard]] Point3D operator*(const Point3D& local_p) const noexcept
    {
        return transform_point(local_p);
    }

    // Returns T_self * point
    // Explicitly says "transform this point," functionally the same as the
    // multiply operator
    [[nodiscard]] Point3D transform_point(const Point3D& point) const noexcept
    {
        const Quaternion normalized_q = q.normalized();

        double num1 = normalized_q.x * 2.0;
        double num2 = normalized_q.y * 2.0;
        double num3 = normalized_q.z * 2.0;
        double num4 = normalized_q.x * num1;
        double num5 = normalized_q.y * num2;
        double num6 = normalized_q.z * num3;
        double num7 = normalized_q.x * num2;
        double num8 = normalized_q.x * num3;
        double num9 = normalized_q.y * num3;
        double num10 = normalized_q.w * num1;
        double num11 = normalized_q.w * num2;
        double num12 = normalized_q.w * num3;

        double transformed_x =
            x_m + ((1.0 - (num5 + num6)) * point.x_m +
                   (num7 - num12) * point.y_m + (num8 + num11) * point.z_m);
        double transformed_y = y_m + ((num7 + num12) * point.x_m +
                                      (1.0 - (num4 + num6)) * point.y_m +
                                      (num9 - num10) * point.z_m);
        double transformed_z =
            z_m + ((num8 - num11) * point.x_m + (num9 + num10) * point.y_m +
                   (1.0 - (num4 + num5)) * point.z_m);

        return Point3D{transformed_x, transformed_y, transformed_z};
    }

    // Conversion: Pose3D -> Pose2D
    // Extracts x, y, and projects the quaternion yaw (rotation around Z) into
    // 2D
    [[nodiscard]] Pose2D to_pose2d() const noexcept
    {
        const Quaternion normalized_q = q.normalized();

        // Extract yaw from quaternion (atan2 of the Z-axis rotation component)
        const double siny_cosp =
            2.0 * (normalized_q.w * normalized_q.z +
                   normalized_q.x * normalized_q.y);
        const double cosy_cosp =
            1.0 - 2.0 * (normalized_q.y * normalized_q.y +
                         normalized_q.z * normalized_q.z);
        double yaw = std::atan2(siny_cosp, cosy_cosp);

        constexpr double pi = 3.14159265358979323846;
        constexpr double two_pi = 2.0 * pi;
        if (yaw >= pi)
        {
            yaw -= two_pi;
        }

        return Pose2D{x_m, y_m, yaw};
    }
};

[[nodiscard]] inline Pose3D Pose2D::to_pose3d() const noexcept
{
    return Pose3D::from_pose2d(*this);
}
}  // namespace transforms
