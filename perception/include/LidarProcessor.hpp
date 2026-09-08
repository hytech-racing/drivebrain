#pragma once

#include <optional>

#include "ClusterFeatures.hpp"
#include "Clustering.hpp"
#include "ConeFilter.hpp"
#include "GroundRemoval.hpp"
#include "PointCloudFilters.hpp"
#include "PointCloudTypes.hpp"

namespace perception
{

struct LidarProcessorParams
{
    bool deskew_enabled{false};
    PointCloudFilterParams point_cloud_filter_params;
    GroundRemovalParams ground_removal_params;
    ClusteringParams clustering_params;
    ConeFilterParams cone_filter_params;
};

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
    std::int64_t timestamp_ns{};

    StampedPointCloud deskewed_point_cloud;
    StampedPointCloud filtered_point_cloud;

    StampedPointCloud ground_point_cloud;
    StampedPointCloud non_ground_point_cloud;

    GroundRemovalDebug ground_removal_debug;

    std::vector<Cluster> clusters;
    ClusteringDebug clustering_debug;

    std::vector<ClusterFeatures> cluster_features;

    std::vector<ConeCandidate> cone_candidates;
    std::vector<RejectedCluster> rejected_clusters;
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
