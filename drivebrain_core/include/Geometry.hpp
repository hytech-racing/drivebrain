#pragma once

#include <Eigen/Geometry>
#include <cmath>

namespace core::geometry
{

[[nodiscard]] inline Eigen::Isometry2d make_pose2d(const double x_m,
                                                   const double y_m,
                                                   const double yaw_rad)
{
    return Eigen::Translation2d(x_m, y_m) * Eigen::Rotation2Dd(yaw_rad);
}

// Wrapped to [-pi, pi]
[[nodiscard]] inline double yaw(const Eigen::Isometry2d& pose)
{
    return Eigen::Rotation2Dd(pose.linear()).angle();
}

// Lifts a planar pose to 3D: z = 0, yaw about +Z
[[nodiscard]] inline Eigen::Isometry3d to_pose3d(const Eigen::Isometry2d& pose)
{
    Eigen::Isometry3d out = Eigen::Isometry3d::Identity();
    out.linear().topLeftCorner<2, 2>() = pose.linear();
    out.translation().head<2>() = pose.translation();
    return out;
}

// SE(2) exponential: the pose reached by moving at a constant twist (vx, vy, yaw rate) for dt_s
[[nodiscard]] inline Eigen::Isometry2d integrate_twist(const Eigen::Vector3d& twist, const double dt_s)
{
    const double theta = twist.z() * dt_s;
    const Eigen::Vector2d v = twist.head<2>() * dt_s;
    const bool straight = std::abs(theta) < 1e-6;
    const double s = straight ? 1.0 : std::sin(theta) / theta;
    const double c = straight ? 0.0 : (1.0 - std::cos(theta)) / theta;
    return make_pose2d(s * v.x() - c * v.y(), c * v.x() + s * v.y(), theta);
}

// Projects a 3D pose onto the XY plane, keeping only yaw
[[nodiscard]] inline Eigen::Isometry2d to_pose2d(const Eigen::Isometry3d& pose)
{
    const Eigen::Matrix3d R = pose.linear();
    return make_pose2d(pose.translation().x(), pose.translation().y(),
                       std::atan2(R(1, 0), R(0, 0)));
}

}  // namespace core::geometry
