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

// Projects a 3D pose onto the XY plane, keeping only yaw
[[nodiscard]] inline Eigen::Isometry2d to_pose2d(const Eigen::Isometry3d& pose)
{
    const Eigen::Matrix3d R = pose.linear();
    return make_pose2d(pose.translation().x(), pose.translation().y(),
                       std::atan2(R(1, 0), R(0, 0)));
}

}  // namespace core::geometry
