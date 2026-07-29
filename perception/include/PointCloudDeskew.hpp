
#pragma once

#include <vector>

#include "PointCloudTypes.hpp"

namespace perception
{
DeskewResult deskew_point_cloud(const StampedPointCloud& stamped_point_cloud,
                                const StampedLidarPose& T_odom_reference,
                                const std::vector<StampedLidarPose>& T_odom_i);
}
