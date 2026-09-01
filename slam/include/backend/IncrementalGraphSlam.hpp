#pragma once

#include <gtsam/base/types.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/ISAM2.h>

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include "IncrementalGraphSlamTypes.hpp"

namespace slam::backend
{

class IncrementalGraphSlam
{
   public:
    IncrementalGraphSlam(const IncrementalGraphSlamParams& params);

    IncrementalGraphSlamResult process_frame(
        const slam::LandmarkFrame& frame);

    IncrementalGraphSlamSnapshot snapshot() const;

   private:
    bool _params_are_ok();

    bool _frame_is_ok(const slam::LandmarkFrame& frame);

   private:
    IncrementalGraphSlamParams _incremental_graph_slam_params;
    std::unique_ptr<gtsam::ISAM2> _isam;

    gtsam::Pose2 _prior_mean;

    gtsam::noiseModel::Diagonal::shared_ptr _prior_noise;
    gtsam::noiseModel::Diagonal::shared_ptr _odom_noise;
    gtsam::noiseModel::Diagonal::shared_ptr _bearing_range_noise;

    // std::set<std::uint64_t> confirmed_landmark_ids_;
    std::set<std::uint64_t> _initialized_landmark_ids;

    std::size_t _next_pose_index{0U};
    std::optional<gtsam::Key> _previous_pose_key;

    std::optional<gtsam::Pose2> _reference_recorded_pose_odom_from_base;
    std::optional<gtsam::Pose2> _previous_recorded_pose_odom_from_base;
    std::optional<std::int64_t> _previous_timestamp_ns;

    std::vector<PoseMetadata> _pose_metadata;
    std::map<std::uint64_t, gtsam::Point2> _initial_landmark_positions_map;

    IncrementalGraphSlamTotals _cumulative_diagnostics;

    bool _failed{false};
};
}  // namespace slam::backend
