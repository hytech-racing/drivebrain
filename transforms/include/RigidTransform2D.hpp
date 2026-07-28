#pragma once

#include <cmath>

namespace transforms
{

struct Point2D
{
    double x_m{};
    double y_m{};
};

struct RigidTransform2D
{
    double x_m{};
    double y_m{};
    double yaw_rad{};

    // Returns T^-1
    [[nodiscard]] RigidTransform2D inverse() const
    {
        const double cos_yaw = std::cos(yaw_rad);
        const double sin_yaw = std::sin(yaw_rad);

        return RigidTransform2D{-x_m * cos_yaw - y_m * sin_yaw,
                                x_m * sin_yaw - y_m * cos_yaw, -yaw_rad};
    }

    // Returns T_self * T_other
    [[nodiscard]] RigidTransform2D operator*(
        const RigidTransform2D& other) const
    {
        const double cos_this = std::cos(yaw_rad);
        const double sin_this = std::sin(yaw_rad);

        double out_yaw = yaw_rad + other.yaw_rad;
        out_yaw = std::atan2(std::sin(out_yaw), std::cos(out_yaw));

        return RigidTransform2D{
            x_m + (other.x_m * cos_this - other.y_m * sin_this),
            y_m + (other.x_m * sin_this + other.y_m * cos_this), out_yaw};
    }

    // Returns T_self * local_p
    // Allows you to write: Point2D global_p = my_transform * local_p
    [[nodiscard]] Point2D operator*(const Point2D& local_p) const
    {
        const double cos_yaw = std::cos(yaw_rad);
        const double sin_yaw = std::sin(yaw_rad);

        return Point2D{x_m + (local_p.x_m * cos_yaw - local_p.y_m * sin_yaw),
                       y_m + (local_p.x_m * sin_yaw + local_p.y_m * cos_yaw)};
    }
};

}  // namespace transforms
