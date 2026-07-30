#pragma once

#include <foxglove/FrameTransform.pb.h>
#include <foxglove/SceneUpdate.pb.h>

#include <cstdint>
#include <memory>
#include <string_view>
#include <vector>

#include "RigidTransform2D.hpp"
#include "common/SlamInterfaces.hpp"

namespace adapters
{

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_slam_pose_markers(
    const std::vector<slam::PoseEstimate>& poses, bool optimized,
    std::int64_t timestamp_ns, std::string_view entity_id);

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_slam_landmark_markers(
    const std::vector<slam::LandmarkEstimate>& landmarks, bool optimized,
    std::int64_t timestamp_ns, std::string_view entity_id);

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_slam_landmark_text(
    const std::vector<slam::LandmarkEstimate>& landmarks,
    std::int64_t timestamp_ns,
    std::string_view entity_id = "slam_landmark_labels");

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_frontend_association_markers(
    const slam::FrontendResult& result, std::string_view frame_id,
    std::string_view entity_id = "slam_frontend_associations");

std::shared_ptr<foxglove::SceneUpdate> to_foxglove_frontend_association_text(
    const slam::FrontendResult& result, std::string_view frame_id,
    std::string_view entity_id = "slam_frontend_association_labels");

std::shared_ptr<foxglove::FrameTransform> to_foxglove_map_odom_transform(
    const transforms::Pose2D& pose_map_from_odom, std::int64_t timestamp_ns);

}  // namespace adapters
