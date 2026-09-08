#pragma once

#include <memory>

#include "LidarProcessor.hpp"
#include "PointCloudTypes.hpp"
#include "dv_msgs.pb.h"

namespace adapters
{

std::shared_ptr<dv_msgs::LidarProcessingDebug> to_lidar_processing_debug(
    const perception::StampedPointCloud& input_cloud,
    const perception::LidarProcessingResult& lidar_processor_result);

}  // namespace adapters
