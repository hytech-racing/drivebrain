#pragma once

#include <cmath>

namespace transforms
{

struct Pose3D;

struct Point2D
{
    double x_m{};
    double y_m{};
};

struct Pose2D
{
    double x_m{};
    double y_m{};
    double yaw_rad{};

    // Returns T^-1
    [[nodiscard]] Pose2D inverse() const
    {
        const double cos_yaw = std::cos(yaw_rad);
        const double sin_yaw = std::sin(yaw_rad);

        return Pose2D{-x_m * cos_yaw - y_m * sin_yaw,
                      x_m * sin_yaw - y_m * cos_yaw, -yaw_rad};
    }

    // Returns T_self * T_other
    [[nodiscard]] Pose2D operator*(const Pose2D& other) const noexcept
    {
        const double cos_this = std::cos(yaw_rad);
        const double sin_this = std::sin(yaw_rad);

        double out_yaw = yaw_rad + other.yaw_rad;
        out_yaw = std::atan2(std::sin(out_yaw), std::cos(out_yaw));

        return Pose2D{x_m + (other.x_m * cos_this - other.y_m * sin_this),
                      y_m + (other.x_m * sin_this + other.y_m * cos_this),
                      out_yaw};
    }

    // Returns T_self * T_other
    // Explicitly says "compose," functionally the same as the multiply operator
    [[nodiscard]] Pose2D compose(const Pose2D& other) const noexcept
    {
        const double cos_this = std::cos(yaw_rad);
        const double sin_this = std::sin(yaw_rad);

        double out_yaw = yaw_rad + other.yaw_rad;
        out_yaw = std::atan2(std::sin(out_yaw), std::cos(out_yaw));

        return Pose2D{x_m + (other.x_m * cos_this - other.y_m * sin_this),
                      y_m + (other.x_m * sin_this + other.y_m * cos_this),
                      out_yaw};
    }

    // Returns T_self * local_p
    // Allows you to write: Point2D global_p = my_transform * local_p
    [[nodiscard]] Point2D operator*(const Point2D& local_p) const noexcept
    {
        const double cos_yaw = std::cos(yaw_rad);
        const double sin_yaw = std::sin(yaw_rad);

        return Point2D{x_m + (local_p.x_m * cos_yaw - local_p.y_m * sin_yaw),
                       y_m + (local_p.x_m * sin_yaw + local_p.y_m * cos_yaw)};
    }

    // Returns T_self * point
    // Explicitly says "transform this point," functionally the same as the
    // multiply operator
    [[nodiscard]] Point2D transform_point(const Point2D& point) const noexcept
    {
        double cy = std::cos(yaw_rad);
        double sy = std::sin(yaw_rad);

        double transformed_x = x_m + (point.x_m * cy - point.y_m * sy);
        double transformed_y = y_m + (point.x_m * sy + point.y_m * cy);

        return Point2D{transformed_x, transformed_y};
    }

    // Conversion: Pose2D -> Pose3D
    // Zeros z-translation, converts yaw rotation to quaternion
    [[nodiscard]] Pose3D to_pose3d() const noexcept;
};

}  // namespace transforms
