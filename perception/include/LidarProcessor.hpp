#pragma once

#include <optional>

#include "PointCloudTypes.hpp"

namespace perception
{

struct LidarMotionContext
{
    StampedLidarPose reference_pose;
    std::vector<StampedLidarPose> point_poses;
};

struct LidarProcessingInput
{
    StampedPointCloud point_cloud;
    std::optional<LidarMotionContext> motion;
};

struct LidarProcessingResult
{
    StampedPointCloud deskewed_point_cloud;
    StampedPointCloud ground_point_cloud;
    StampedPointCloud non_ground_point_cloud;
};

class LidarProcessor
{
   public:
    LidarProcessor(const LidarProcessorParams& params);

    std::optional<LidarProcessingResult> process(
        const LidarProcessingInput& input);

   private:
    LidarProcessorParams _params;
};

}  // namespace perception
