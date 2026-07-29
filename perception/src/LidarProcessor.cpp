#include "LidarProcessor.hpp"

#include <spdlog/spdlog.h>

#include <utility>

#include "ClusterFeatures.hpp"
#include "Clustering.hpp"
#include "ConeFilter.hpp"
#include "GroundRemoval.hpp"
#include "PointCloudDeskew.hpp"
#include "PointCloudFilters.hpp"

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

    if (_params.deskew_enabled && !input.motion)
    {
        spdlog::warn(
            "Rejected lidar point cloud: deskew is enabled but no motion "
            "context was provided");
        return std::nullopt;
    }

    if (_params.deskew_enabled)
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

    StampedPointCloud filtered_point_cloud = filter_point_cloud(
        deskew_result.stamped_point_cloud, _params.point_cloud_filter_params);

    GroundRemovalResult ground_removal_result =
        remove_ground(filtered_point_cloud, _params.ground_removal_params);

    ClusteringResult clustering_result = euclidean_cluster_xy(
        ground_removal_result.non_ground_points, _params.clustering_params);

    std::vector<ClusterFeatures> cluster_features =
        compute_all_cluster_features(ground_removal_result.non_ground_points,
                                     clustering_result.clusters);
    ConeFilterResult cone_filter_result =
        filter_cone_candidates(cluster_features, _params.cone_filter_params);

    LidarProcessingResult result;
    result.deskewed_point_cloud = std::move(deskew_result.stamped_point_cloud);
    result.filtered_point_cloud = std::move(filtered_point_cloud);
    result.ground_point_cloud = std::move(ground_removal_result.ground_points);
    result.non_ground_point_cloud =
        std::move(ground_removal_result.non_ground_points);
    result.ground_removal_debug = std::move(ground_removal_result.debug);
    result.clusters = std::move(clustering_result.clusters);
    result.clustering_debug = clustering_result.debug;
    result.cluster_features = std::move(cluster_features);
    result.cone_candidates = std::move(cone_filter_result.candidates);
    result.rejected_clusters = std::move(cone_filter_result.rejected);

    return result;
}

}  // namespace perception
