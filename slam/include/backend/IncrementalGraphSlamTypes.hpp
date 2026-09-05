#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "common/SlamInterfaces.hpp"
#include "RigidTransform2D.hpp"

namespace slam::backend
{

struct Isam2TuningParams
{
    double relinearization_threshold{0.1};
    std::size_t relinearization_skip{10U};

    // Enables error_before_update and error_after_update diagnostics.
    bool evaluate_nonlinear_error{false};
};

struct IncrementalGraphSlamParams
{
    slam::PriorPoseNoise prior_pose_noise{};
    slam::OdometryPoseNoise odom_pose_noise{};
    slam::LandmarkMeasurementNoise landmark_measurement_noise{};
    slam::MeasurementRange measurement_range{};

    Isam2TuningParams isam2{};
};

struct ObservationRejectionCounts
{
    std::size_t unconfirmed_landmark_id{};
    std::size_t nonfinite_measurement{};
    std::size_t outside_measurement_range{};
    std::size_t duplicate_landmark_id{};

    [[nodiscard]] std::size_t total() const noexcept
    {
        return unconfirmed_landmark_id + nonfinite_measurement +
               outside_measurement_range + duplicate_landmark_id;
    }
};

struct IncrementalUpdateCounts
{
    std::size_t observations_received{};
    std::size_t observations_admitted{};
    ObservationRejectionCounts observations_rejected{};

    std::size_t new_landmarks_added{};
    std::size_t new_values_added{};

    std::size_t prior_factors_added{};
    std::size_t between_factors_added{};
    std::size_t measurement_factors_added{};

    [[nodiscard]] std::size_t factors_added() const noexcept
    {
        return prior_factors_added + between_factors_added +
               measurement_factors_added;
    }
};

struct IncrementalGraphSlamTotals
{
    std::size_t pose_count{};
    std::size_t landmark_count{};

    std::size_t prior_factor_count{};
    std::size_t between_factor_count{};
    std::size_t measurement_factor_count{};

    std::size_t admitted_observation_count{};
    ObservationRejectionCounts rejected_observations{};

    [[nodiscard]] std::size_t factor_count() const noexcept
    {
        return prior_factor_count + between_factor_count +
               measurement_factor_count;
    }
};

struct Isam2UpdateDiagnostics
{
    std::optional<double> error_before_update{};
    std::optional<double> error_after_update{};

    std::size_t variables_relinearized{};
    std::size_t variables_reeliminated{};
};

struct IncrementalPoseResult
{
    std::size_t pose_index{};
    std::uint64_t frame_index{};
    std::int64_t timestamp_ns{};

    // T_odom_base from the incoming frame
    transforms::Pose2D recorded_pose_odom_from_base{};

    // initial guess inserted for X(k)
    transforms::Pose2D initial_pose_map_from_base{};

    // current estimate of X(k) after the isam2 update
    transforms::Pose2D optimized_pose_map_from_base{};

    // T_map_odom calculated from the optimized and recorded poses
    transforms::Pose2D pose_map_from_odom{};
};

struct IncrementalGraphSlamDebug
{
    // false when frame-level validation rejects the frame
    bool frame_accepted{};

    // true only when isam2 update and result extraction succeed
    bool update_success{};

    // true when an optimizer failure occurs
    bool core_failed{};

    std::string message{};

    IncrementalUpdateCounts update{};
    IncrementalGraphSlamTotals cumulative{};
    Isam2UpdateDiagnostics isam2{};
};

struct IncrementalGraphSlamResult
{
    IncrementalGraphSlamDebug debug{};

    // empty when the frame was rejected or the update failed
    std::optional<IncrementalPoseResult> current_pose{};

    // only landmarks introduced by this update
    std::vector<slam::LandmarkEstimate> new_landmarks{};
};

struct PoseMetadata
{
    std::size_t pose_index{};
    std::uint64_t frame_index{};
    std::int64_t timestamp_ns{};

    transforms::Pose2D recorded_pose_odom_from_base{};
    transforms::Pose2D initial_pose_map_from_base{};
};

struct IncrementalGraphSlamSnapshot
{
    bool success{};
    std::string message{};

    IncrementalGraphSlamTotals totals{};

    // complete estimates at the time snapshot() is called.
    std::vector<slam::PoseEstimate> poses{};
    std::vector<slam::LandmarkEstimate> landmarks{};

    std::optional<transforms::Pose2D> latest_pose_map_from_odom{};
};
}  // namespace slam::backend
