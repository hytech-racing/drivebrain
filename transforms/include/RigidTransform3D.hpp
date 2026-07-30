#pragma once
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

    [[nodiscard]] Quaternion multiply(const Quaternion& q) const noexcept
    {
        double rw = w * q.w - x * q.x - y * q.y - z * q.z;
        double rx = w * q.x + x * q.w + y * q.z - z * q.y;
        double ry = w * q.y - x * q.z + y * q.w + z * q.x;
        double rz = w * q.z + x * q.y - y * q.x + z * q.w;

        double len = std::sqrt(rw * rw + rx * rx + ry * ry + rz * rz);
        if (len > 1e-9)
        {
            rw /= len;
            rx /= len;
            ry /= len;
            rz /= len;
        }

        return Quaternion{rw, rx, ry, rz};
    }

    [[nodiscard]] Quaternion operator*(const Quaternion& other) const noexcept
    {
        return multiply(other);
    }

    [[nodiscard]] Quaternion conjugate() const noexcept
    {
        return Quaternion{w, -x, -y, -z};
    }
};

struct Pose3D
{
    double x_m{};
    double y_m{};
    double z_m{};
    Quaternion q{};

    // Returns T^-1
    [[nodiscard]] Pose3D inverse() const noexcept
    {
        Quaternion inv_q = q.conjugate();

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
        double num1 = q.x * 2.0;
        double num2 = q.y * 2.0;
        double num3 = q.z * 2.0;
        double num4 = q.x * num1;
        double num5 = q.y * num2;
        double num6 = q.z * num3;
        double num7 = q.x * num2;
        double num8 = q.x * num3;
        double num9 = q.y * num3;
        double num10 = q.w * num1;
        double num11 = q.w * num2;
        double num12 = q.w * num3;

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
        double num1 = q.x * 2.0;
        double num2 = q.y * 2.0;
        double num3 = q.z * 2.0;
        double num4 = q.x * num1;
        double num5 = q.y * num2;
        double num6 = q.z * num3;
        double num7 = q.x * num2;
        double num8 = q.x * num3;
        double num9 = q.y * num3;
        double num10 = q.w * num1;
        double num11 = q.w * num2;
        double num12 = q.w * num3;

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
        // Extract yaw from quaternion (atan2 of the Z-axis rotation component)
        const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        const double yaw = std::atan2(siny_cosp, cosy_cosp);

        return Pose2D{x_m, y_m, yaw};
    }
};

[[nodiscard]] inline Pose3D Pose2D::to_pose3d() const noexcept
{
    const double half_yaw = yaw_rad * 0.5;

    Quaternion quaternion_z;
    quaternion_z.w = std::cos(half_yaw);
    quaternion_z.x = 0.0;
    quaternion_z.y = 0.0;
    quaternion_z.z = std::sin(half_yaw);

    return Pose3D{x_m, y_m, 0.0, quaternion_z};
}
}  // namespace transforms
