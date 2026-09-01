#include "SlamDebugMessageAdapters.hpp"

#include <cstddef>
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

void set_pose2d(dv_msgs::IncrementalGraphSlamDebug* message,
                const slam::backend::IncrementalPoseResult& pose)
{
    message->set_pose_index(to_debug_count(pose.pose_index));

    message->set_recorded_pose_odom_x_m(pose.recorded_pose_odom_from_base.x_m);
    message->set_recorded_pose_odom_y_m(pose.recorded_pose_odom_from_base.y_m);
    message->set_recorded_pose_odom_yaw_rad(
        pose.recorded_pose_odom_from_base.yaw_rad);

    message->set_initial_pose_map_x_m(pose.initial_pose_map_from_base.x_m);
    message->set_initial_pose_map_y_m(pose.initial_pose_map_from_base.y_m);
    message->set_initial_pose_map_yaw_rad(
        pose.initial_pose_map_from_base.yaw_rad);

    message->set_optimized_pose_map_x_m(pose.optimized_pose_map_from_base.x_m);
    message->set_optimized_pose_map_y_m(pose.optimized_pose_map_from_base.y_m);
    message->set_optimized_pose_map_yaw_rad(
        pose.optimized_pose_map_from_base.yaw_rad);

    message->set_pose_map_from_odom_x_m(pose.pose_map_from_odom.x_m);
    message->set_pose_map_from_odom_y_m(pose.pose_map_from_odom.y_m);
    message->set_pose_map_from_odom_yaw_rad(pose.pose_map_from_odom.yaw_rad);
}

void set_rejection_counts(
    dv_msgs::SlamObservationRejectionCounts* message,
    const slam::backend::ObservationRejectionCounts& counts)
{
    message->set_unconfirmed_landmark_id(
        to_debug_count(counts.unconfirmed_landmark_id));
    message->set_nonfinite_measurement(
        to_debug_count(counts.nonfinite_measurement));
    message->set_outside_measurement_range(
        to_debug_count(counts.outside_measurement_range));
    message->set_duplicate_landmark_id(
        to_debug_count(counts.duplicate_landmark_id));
}

void set_update_counts(dv_msgs::IncrementalGraphSlamUpdateCounts* message,
                       const slam::backend::IncrementalUpdateCounts& counts)
{
    message->set_observations_received(
        to_debug_count(counts.observations_received));
    message->set_observations_admitted(
        to_debug_count(counts.observations_admitted));
    set_rejection_counts(message->mutable_observations_rejected(),
                         counts.observations_rejected);

    message->set_new_landmarks_added(
        to_debug_count(counts.new_landmarks_added));
    message->set_new_values_added(to_debug_count(counts.new_values_added));
    message->set_prior_factors_added(
        to_debug_count(counts.prior_factors_added));
    message->set_between_factors_added(
        to_debug_count(counts.between_factors_added));
    message->set_measurement_factors_added(
        to_debug_count(counts.measurement_factors_added));
}

void set_totals(dv_msgs::IncrementalGraphSlamTotals* message,
                const slam::backend::IncrementalGraphSlamTotals& totals)
{
    message->set_pose_count(to_debug_count(totals.pose_count));
    message->set_landmark_count(to_debug_count(totals.landmark_count));
    message->set_prior_factor_count(to_debug_count(totals.prior_factor_count));
    message->set_between_factor_count(
        to_debug_count(totals.between_factor_count));
    message->set_measurement_factor_count(
        to_debug_count(totals.measurement_factor_count));
    message->set_admitted_observation_count(
        to_debug_count(totals.admitted_observation_count));
    set_rejection_counts(message->mutable_rejected_observations(),
                         totals.rejected_observations);
}

void set_isam2(dv_msgs::Isam2UpdateDiagnostics* message,
               const slam::backend::Isam2UpdateDiagnostics& diagnostics)
{
    if (diagnostics.error_before_update)
    {
        message->set_has_error_before_update(true);
        message->set_error_before_update(*diagnostics.error_before_update);
    }

    if (diagnostics.error_after_update)
    {
        message->set_has_error_after_update(true);
        message->set_error_after_update(*diagnostics.error_after_update);
    }

    message->set_variables_relinearized(
        to_debug_count(diagnostics.variables_relinearized));
    message->set_variables_reeliminated(
        to_debug_count(diagnostics.variables_reeliminated));
}

}  // namespace

std::shared_ptr<dv_msgs::SlamFrontendDebug> to_slam_frontend_debug(
    const slam::FrontendResult& result)
{
    auto message = std::make_shared<dv_msgs::SlamFrontendDebug>();

    message->set_timestamp_ns(result.timestamp_ns);
    message->set_frame_accepted(result.frame_accepted);
    message->set_message(result.message);
    message->set_landmark_observations(
        to_debug_count(result.landmark_observations.size()));

    for (const std::uint64_t landmark_id : result.new_landmark_ids)
    {
        message->add_new_landmark_ids(landmark_id);
    }

    dv_msgs::SlamFrontendDiagnostics* diagnostics =
        message->mutable_diagnostics();
    diagnostics->set_detections_received(
        to_debug_count(result.debug.detections_received));
    diagnostics->set_detections_rejected_invalid(
        to_debug_count(result.debug.detections_rejected_invalid));
    diagnostics->set_optimized_associations(
        to_debug_count(result.debug.optimized_associations));
    diagnostics->set_pending_associations(
        to_debug_count(result.debug.pending_associations));
    diagnostics->set_tentative_associations(
        to_debug_count(result.debug.tentative_associations));
    diagnostics->set_tentative_tracks_created(
        to_debug_count(result.debug.tentative_tracks_created));
    diagnostics->set_tracks_promoted(
        to_debug_count(result.debug.tracks_promoted));
    diagnostics->set_tracks_removed_stale(
        to_debug_count(result.debug.tracks_removed_stale));
    diagnostics->set_unmatched_detection_count(
        to_debug_count(result.debug.unmatched_detection_count));

    return message;
}

std::shared_ptr<dv_msgs::IncrementalGraphSlamDebug>
to_incremental_graph_slam_debug(
    const slam::LandmarkFrame& frame,
    const slam::backend::IncrementalGraphSlamResult& result)
{
    auto message = std::make_shared<dv_msgs::IncrementalGraphSlamDebug>();

    message->set_timestamp_ns(frame.timestamp_ns);
    message->set_frame_index(frame.frame_index);
    message->set_frame_accepted(result.debug.frame_accepted);
    message->set_update_success(result.debug.update_success);
    message->set_core_failed(result.debug.core_failed);
    message->set_message(result.debug.message);

    set_update_counts(message->mutable_update(), result.debug.update);
    set_totals(message->mutable_cumulative(), result.debug.cumulative);
    set_isam2(message->mutable_isam2(), result.debug.isam2);

    if (result.current_pose)
    {
        message->set_has_current_pose(true);
        set_pose2d(message.get(), *result.current_pose);
    }

    return message;
}

}  // namespace adapters
