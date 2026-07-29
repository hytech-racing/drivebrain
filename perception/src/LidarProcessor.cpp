#include "LidarProcessor.hpp"

#include "GroundRemoval.hpp"
#include "PointCloudDeskew.hpp"

namespace perception
{

LidarProcessor::LidarProcessor(const LidarProcessorParams& params)
    : _params(params)
{
}

std::optional<LidarProcessingResult> LidarProcessor::process(
    const LidarProcessingInput& input)
{
    if (input.point_cloud.frame != transforms::FrameId::Lidar)
    {
        return std::nullopt;
    }

    DeskewResult deskew_result;

    if (_params.deskew_enabled && input.motion)
    {
        deskew_result =
            deskew_point_cloud(input.point_cloud, input.motion->reference_pose,
                               input.motion->point_poses);
    }
    else
    {
        deskew_result.stamped_point_cloud = input.point_cloud;
        deskew_result.start_stamp_ns =
            deskew_result.stamped_point_cloud.timestamp_ns;
        deskew_result.end_stamp_ns =
            deskew_result.stamped_point_cloud.timestamp_ns;
        deskew_result.reference_stamp_ns =
            deskew_result.stamped_point_cloud.timestamp_ns;
    }

    GroundRemovalResult ground_removal_result =
        remove_ground(deskew_result.stamped_point_cloud, GroundRemovalParams{});

    LidarProcessingResult result;
    result.deskewed_point_cloud = deskew_result.stamped_point_cloud;
    result.ground_point_cloud = ground_removal_result.ground_points;
    result.non_ground_point_cloud = ground_removal_result.non_ground_points;

    return result;
}

}  // namespace perception
