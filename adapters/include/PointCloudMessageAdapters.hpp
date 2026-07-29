#pragma once

#include <foxglove/PointCloud.pb.h>
#include <foxglove/SceneUpdate.pb.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

#include "ClusterFeatures.hpp"
#include "ConeFilter.hpp"
#include "PointCloudTypes.hpp"

namespace adapters
{

std::optional<perception::StampedPointCloud> to_core_point_cloud(
    const foxglove::PointCloud& message);

std::shared_ptr<foxglove::PointCloud> to_foxglove_point_cloud(
    const perception::StampedPointCloud& cloud, std::string_view frame_id);

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_cluster_markers(
    const std::vector<perception::ClusterFeatures>& cluster_features,
    std::string_view frame_id, std::int64_t timestamp_ns,
    std::string_view entity_id = "clusters");

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_cone_candidate_markers(
    const std::vector<perception::ConeCandidate>& cone_candidates,
    std::string_view frame_id, std::int64_t timestamp_ns,
    std::string_view entity_id = "cone_candidates");

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_cone_candidate_text(
    const std::vector<perception::ConeCandidate>& cone_candidates,
    std::string_view frame_id, std::int64_t timestamp_ns,
    std::string_view entity_id = "cone_candidate_debug_text");

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_rejected_cluster_markers(
    const std::vector<perception::RejectedCluster>& rejected_clusters,
    std::string_view frame_id, std::int64_t timestamp_ns,
    std::string_view entity_id = "rejected_clusters");

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_rejected_cluster_text(
    const std::vector<perception::RejectedCluster>& rejected_clusters,
    std::string_view frame_id, std::int64_t timestamp_ns,
    std::string_view entity_id = "rejection_reason_text");

}
