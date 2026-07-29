#pragma once

#include <cmath>
#include <cstdint>
#include <vector>

#include "FrameId.hpp"
#include "RigidTransform3D.hpp"

namespace perception
{

struct PointXYZI
{
    float x{};
    float y{};
    float z{};
    float intensity{};
};

using PointCloud = std::vector<PointXYZI>;

struct StampedLidarPose
{
    std::int64_t stamp_ns{};
    transforms::Pose3D pose{};
};

struct StampedPointCloud
{
    std::int64_t timestamp_ns{};
    transforms::FrameId frame{transforms::FrameId::Lidar};
    PointCloud points;
};

struct DeskewResult
{
    std::int64_t start_stamp_ns{};
    std::int64_t end_stamp_ns{};
    std::int64_t reference_stamp_ns{};
    StampedPointCloud stamped_point_cloud;
};

[[nodiscard]] PointXYZI transform_point(const transforms::Pose3D& transform,
                                        const PointXYZI& point) noexcept;

}  // namespace perception
