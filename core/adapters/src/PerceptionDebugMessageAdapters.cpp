#include "PerceptionDebugMessageAdapters.hpp"

#include <cstdint>
#include <limits>

namespace adapters
{
namespace
{

std::uint32_t to_debug_count(const std::size_t count)
{
    return count > std::numeric_limits<std::uint32_t>::max()
               ? std::numeric_limits<std::uint32_t>::max()
               : static_cast<std::uint32_t>(count);
}

void set_ground_removal_debug(dv_msgs::GroundRemovalDebug* message,
                              const perception::GroundRemovalDebug& debug)
{
    message->set_input_points(to_debug_count(debug.input_points));
    message->set_ground_points(to_debug_count(debug.ground_points));
    message->set_non_ground_points(to_debug_count(debug.non_ground_points));
    message->set_valid_cells(to_debug_count(debug.valid_cells));
    message->set_invalid_cells(to_debug_count(debug.invalid_cells));
}

void set_clustering_debug(dv_msgs::ClusteringDebug* message,
                          const perception::ClusteringDebug& debug)
{
    message->set_input_points(to_debug_count(debug.input_points));
    message->set_raw_cluster_count(to_debug_count(debug.raw_cluster_count));
    message->set_accepted_cluster_count(
        to_debug_count(debug.accepted_cluster_count));
    message->set_rejected_too_small(to_debug_count(debug.rejected_too_small));
    message->set_rejected_too_large(to_debug_count(debug.rejected_too_large));
}

void set_cone_filter_debug(
    dv_msgs::ConeFilterDebug* message,
    const perception::LidarProcessingResult& lidar_processor_result)
{
    message->set_input_clusters(
        to_debug_count(lidar_processor_result.cluster_features.size()));
    message->set_accepted_candidates(
        to_debug_count(lidar_processor_result.cone_candidates.size()));
    message->set_rejected_clusters(
        to_debug_count(lidar_processor_result.rejected_clusters.size()));

    for (const auto& rejected_cluster :
         lidar_processor_result.rejected_clusters)
    {
        switch (rejected_cluster.reason)
        {
            case perception::ConeRejectionReason::None:
                break;
            case perception::ConeRejectionReason::TooFar:
                message->set_rejected_too_far(message->rejected_too_far() + 1U);
                break;
            case perception::ConeRejectionReason::TooFewPoints:
                message->set_rejected_too_few_points(
                    message->rejected_too_few_points() + 1U);
                break;
            case perception::ConeRejectionReason::TooShort:
                message->set_rejected_too_short(message->rejected_too_short() +
                                                1U);
                break;
            case perception::ConeRejectionReason::TooTall:
                message->set_rejected_too_tall(message->rejected_too_tall() +
                                               1U);
                break;
            case perception::ConeRejectionReason::TooWide:
                message->set_rejected_too_wide(message->rejected_too_wide() +
                                               1U);
                break;
            case perception::ConeRejectionReason::TooElongated:
                message->set_rejected_too_elongated(
                    message->rejected_too_elongated() + 1U);
                break;
        }
    }
}

}  // namespace

std::shared_ptr<dv_msgs::LidarProcessingDebug> to_lidar_processing_debug(
    const perception::StampedPointCloud& input_cloud,
    const perception::LidarProcessingResult& lidar_processor_result)
{
    auto message = std::make_shared<dv_msgs::LidarProcessingDebug>();

    message->set_timestamp_ns(
        static_cast<std::uint64_t>(input_cloud.timestamp_ns));
    message->set_input_points(to_debug_count(input_cloud.points.size()));
    message->set_deskewed_points(to_debug_count(
        lidar_processor_result.deskewed_point_cloud.points.size()));
    message->set_filtered_points(to_debug_count(
        lidar_processor_result.filtered_point_cloud.points.size()));
    message->set_ground_points(to_debug_count(
        lidar_processor_result.ground_point_cloud.points.size()));
    message->set_non_ground_points(to_debug_count(
        lidar_processor_result.non_ground_point_cloud.points.size()));

    set_ground_removal_debug(message->mutable_ground_removal(),
                             lidar_processor_result.ground_removal_debug);
    set_clustering_debug(message->mutable_clustering(),
                         lidar_processor_result.clustering_debug);
    set_cone_filter_debug(message->mutable_cone_filter(),
                          lidar_processor_result);

    return message;
}

}  // namespace adapters
