
#include "backend/IncrementalGraphSlam.hpp"

#include <gtsam/geometry/Rot2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <cmath>
#include <exception>
#include <stdexcept>

namespace slam::backend
{

namespace
{

gtsam::Pose2 to_gtsam_pose(const transforms::Pose2D& pose)
{
    return gtsam::Pose2{pose.x_m, pose.y_m, pose.yaw_rad};
}

gtsam::Point2 to_gtsam_point(const transforms::Point2D& point)
{
    return gtsam::Point2{point.x_m, point.y_m};
}

transforms::Pose2D to_transform_pose(const gtsam::Pose2& pose)
{
    return transforms::Pose2D{pose.x(), pose.y(), pose.theta()};
}

transforms::Point2D to_transform_point(const gtsam::Point2& point)
{
    return transforms::Point2D{point.x(), point.y()};
}

}  // namespace

IncrementalGraphSlam::IncrementalGraphSlam(
    const IncrementalGraphSlamParams& params)
    : _incremental_graph_slam_params(params)
{
    if (!_params_are_ok())
    {
        throw std::invalid_argument("Invalid incremental GraphSLAM parameters");
    }

    gtsam::ISAM2Params isam2_params;

    isam2_params.relinearizeThreshold =
        params.isam2.relinearization_threshold;  // default 0.1
    isam2_params.relinearizeSkip =
        params.isam2.relinearization_skip;  // deault 10
    isam2_params.evaluateNonlinearError =
        params.isam2.evaluate_nonlinear_error;  // default false, want true

    _isam = std::make_unique<gtsam::ISAM2>(isam2_params);

    _prior_mean = gtsam::Pose2(0.0, 0.0, 0.0);

    _prior_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(
        _incremental_graph_slam_params.prior_pose_noise.x_std_m,
        _incremental_graph_slam_params.prior_pose_noise.y_std_m,
        _incremental_graph_slam_params.prior_pose_noise.yaw_std_rad));

    _odom_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(
        _incremental_graph_slam_params.odom_pose_noise.x_std_m,
        _incremental_graph_slam_params.odom_pose_noise.y_std_m,
        _incremental_graph_slam_params.odom_pose_noise.yaw_std_rad));

    _bearing_range_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(
        _incremental_graph_slam_params.landmark_measurement_noise
            .bearing_std_rad,
        _incremental_graph_slam_params.landmark_measurement_noise.range_std_m));
}

IncrementalGraphSlamResult IncrementalGraphSlam::process_frame(
    const LandmarkFrame& frame)
{
    IncrementalGraphSlamResult result;

    if (_failed)
    {
        result.debug.frame_accepted = false;
        result.debug.core_failed = true;
        result.debug.cumulative = _cumulative_diagnostics;
        result.debug.message = "Core failed";
        return result;
    }

    if (!_frame_is_ok(frame))
    {
        result.debug.frame_accepted = false;
        result.debug.message = "Frame rejected";
        result.debug.cumulative = _cumulative_diagnostics;
        return result;
    }

    result.debug.frame_accepted = true;

    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;

    const gtsam::Pose2 recorded_pose_odom_from_base =
        to_gtsam_pose(frame.recorded_pose_odom_from_base);

    const std::size_t current_pose_index = _next_pose_index;

    gtsam::Key current_pose_key =
        gtsam::symbol_shorthand::X(current_pose_index);

    const gtsam::Pose2 reference_pose_odom =
        _reference_recorded_pose_odom_from_base.value_or(
            recorded_pose_odom_from_base);

    const bool introduces_reference =
        !_reference_recorded_pose_odom_from_base.has_value();

    if (!_reference_recorded_pose_odom_from_base)
    {
        new_factors.add(gtsam::PriorFactor<gtsam::Pose2>(
            current_pose_key, _prior_mean, _prior_noise));
        result.debug.update.prior_factors_added++;
    }

    const gtsam::Pose2 recorded_rebased_pose_map =
        reference_pose_odom.between(recorded_pose_odom_from_base);

    gtsam::Pose2 current_pose_initial_guess_map = recorded_rebased_pose_map;

    if (_previous_pose_key && _previous_recorded_pose_odom_from_base)
    {
        const gtsam::Pose2 relative_odom_measurement =
            _previous_recorded_pose_odom_from_base->between(
                recorded_pose_odom_from_base);

        gtsam::Pose2 previous_optimized_pose_map;

        try
        {
            previous_optimized_pose_map =
                _isam->calculateEstimate<gtsam::Pose2>(
                    _previous_pose_key.value());
        }
        catch (const std::exception& e)
        {
            result.debug.update_success = false;
            result.debug.core_failed = true;
            result.debug.message =
                std::string("Failed to retrieve previous optimized pose: ") +
                e.what();
            result.debug.cumulative = _cumulative_diagnostics;

            _failed = true;
            return result;
        }

        current_pose_initial_guess_map =
            previous_optimized_pose_map.compose(relative_odom_measurement);

        new_factors.add(gtsam::BetweenFactor<gtsam::Pose2>(
            _previous_pose_key.value(), current_pose_key,
            relative_odom_measurement, _odom_noise));

        result.debug.update.between_factors_added++;
    }
    new_values.insert(current_pose_key, current_pose_initial_guess_map);

    result.debug.update.new_values_added++;

    result.debug.update.observations_received = frame.observations.size();

    std::set<std::uint64_t> staged_new_landmark_ids;

    std::map<std::uint64_t, gtsam::Point2> staged_initial_landmark_positions;

    std::set<std::uint64_t> seen_mapper_ids_this_frame;
    for (const LandmarkObservation& observation : frame.observations)
    {
        const gtsam::Point2 measurement_base_m =
            to_gtsam_point(observation.measurement_base_m);

        if (!std::isfinite(measurement_base_m.x()) ||
            !std::isfinite(measurement_base_m.y()))
        {
            result.debug.update.observations_rejected.nonfinite_measurement++;
            continue;
        }

        const double measurement_range =
            std::hypot(measurement_base_m.x(), measurement_base_m.y());

        if (measurement_range <
                _incremental_graph_slam_params.measurement_range.min_m ||
            measurement_range >
                _incremental_graph_slam_params.measurement_range.max_m)
        {
            result.debug.update.observations_rejected
                .outside_measurement_range++;
            continue;
        }

        const double bearing_rad =
            std::atan2(measurement_base_m.y(), measurement_base_m.x());

        if (seen_mapper_ids_this_frame.count(observation.landmark_id))
        {
            result.debug.update.observations_rejected.duplicate_landmark_id++;
            continue;
        }

        const gtsam::Key landmark_key =
            gtsam::symbol_shorthand::L(observation.landmark_id);
        seen_mapper_ids_this_frame.insert(observation.landmark_id);

        const bool already_committed =
            _initialized_landmark_ids.count(observation.landmark_id) > 0;

        const bool already_staged =
            staged_new_landmark_ids.count(observation.landmark_id) > 0;

        const bool known_landmark = already_committed || already_staged;

        if (observation.association == LandmarkAssociation::NewLandmark &&
            known_landmark)
        {
            result.debug.update.observations_rejected.duplicate_landmark_id++;
            continue;
        }

        if (observation.association != LandmarkAssociation::NewLandmark &&
            !known_landmark)
        {
            result.debug.update.observations_rejected.unconfirmed_landmark_id++;
            continue;
        }

        if (observation.association == LandmarkAssociation::NewLandmark)
        {
            const gtsam::Point2 initial_position =
                current_pose_initial_guess_map.transformFrom(
                    measurement_base_m);

            new_values.insert(landmark_key, initial_position);

            staged_new_landmark_ids.insert(observation.landmark_id);

            staged_initial_landmark_positions.emplace(observation.landmark_id,
                                                      initial_position);

            result.debug.update.new_landmarks_added++;
            result.debug.update.new_values_added++;
        }

        new_factors.add(gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>(
            current_pose_key, landmark_key, gtsam::Rot2(bearing_rad),
            measurement_range, _bearing_range_noise));
        result.debug.update.observations_admitted++;
        result.debug.update.measurement_factors_added++;
    }

    std::vector<std::pair<gtsam::Key, gtsam::Point2>> new_landmarks_before;

    for (const gtsam::Values::ConstKeyValuePair& key_value : new_values)
    {
        gtsam::Key key = key_value.key;
        gtsam::Symbol symbol(key);

        if (symbol.chr() == 'l')
        {
            gtsam::Point2 initial_guess = key_value.value.cast<gtsam::Point2>();
            new_landmarks_before.push_back({key, initial_guess});
        }
    }

    const PoseMetadata current_pose_metadata{
        current_pose_index,
        frame.frame_index,
        frame.timestamp_ns,
        to_transform_pose(recorded_pose_odom_from_base),
        to_transform_pose(current_pose_initial_guess_map),
    };

    gtsam::ISAM2Result isam_result;

    try
    {
        isam_result = _isam->update(new_factors, new_values);
    }
    catch (const std::exception& e)
    {
        result.debug.update_success = false;
        result.debug.message =
            std::string("iSAM2 update threw exception: ") + e.what();
        result.debug.core_failed = true;

        result.debug.cumulative = _cumulative_diagnostics;
        _failed = true;
        return result;
    }

    result.debug.isam2.variables_relinearized =
        isam_result.variablesRelinearized;
    result.debug.isam2.variables_reeliminated =
        isam_result.variablesReeliminated;

    if (isam_result.errorBefore)
    {
        result.debug.isam2.error_before_update = *isam_result.errorBefore;
    }

    if (isam_result.errorAfter)
    {
        result.debug.isam2.error_after_update = *isam_result.errorAfter;
    }

    gtsam::Values current_estimate;
    try
    {
        current_estimate = _isam->calculateBestEstimate();
    }
    catch (const std::exception& e)
    {
        result.debug.update_success = false;
        result.debug.message =
            std::string("iSAM2 calculateBestEstimate threw exception: ") +
            e.what();
        result.debug.core_failed = true;
        result.debug.cumulative = _cumulative_diagnostics;
        _failed = true;
        return result;
    }

    try
    {
        gtsam::Pose2 optimized_pose_map =
            current_estimate.at<gtsam::Pose2>(current_pose_key);

        gtsam::Pose2 latest_pose_map_from_odom =
            optimized_pose_map.compose(recorded_pose_odom_from_base.inverse());
        result.current_pose =
            IncrementalPoseResult{current_pose_metadata.pose_index,
                                  current_pose_metadata.frame_index,
                                  current_pose_metadata.timestamp_ns,
                                  current_pose_metadata.recorded_pose_odom_from_base,
                                  current_pose_metadata.initial_pose_map_from_base,
                                  to_transform_pose(optimized_pose_map),
                                  to_transform_pose(latest_pose_map_from_odom)};

        result.new_landmarks.reserve(result.debug.update.new_landmarks_added);

        for (const auto& landmark_pair : new_landmarks_before)
        {
            gtsam::Key key = landmark_pair.first;
            gtsam::Symbol symbol(key);

            gtsam::Point2 initial_position_map = landmark_pair.second;
            gtsam::Point2 optimized_position_map =
                current_estimate.at<gtsam::Point2>(key);

            const LandmarkEstimate landmark_est{
                symbol.index(), to_transform_point(initial_position_map),
                to_transform_point(optimized_position_map)};
            result.new_landmarks.push_back(landmark_est);
        }
    }
    catch (const std::exception& e)
    {
        result.debug.update_success = false;
        result.debug.message =
            std::string("iSAM2 estimate extraction threw exception: ") +
            e.what();
        result.debug.core_failed = true;
        result.debug.cumulative = _cumulative_diagnostics;
        _failed = true;
        return result;
    }

    _pose_metadata.push_back(current_pose_metadata);

    _initialized_landmark_ids.insert(staged_new_landmark_ids.begin(),
                                     staged_new_landmark_ids.end());

    _initial_landmark_positions_map.insert(
        staged_initial_landmark_positions.begin(),
        staged_initial_landmark_positions.end());

    if (introduces_reference)
    {
        _reference_recorded_pose_odom_from_base = reference_pose_odom;
    }

    _cumulative_diagnostics.pose_count++;

    _cumulative_diagnostics.prior_factor_count +=
        result.debug.update.prior_factors_added;
    _cumulative_diagnostics.between_factor_count +=
        result.debug.update.between_factors_added;
    _cumulative_diagnostics.measurement_factor_count +=
        result.debug.update.measurement_factors_added;

    _cumulative_diagnostics.landmark_count +=
        result.debug.update.new_landmarks_added;
    _cumulative_diagnostics.admitted_observation_count +=
        result.debug.update.observations_admitted;

    _cumulative_diagnostics.rejected_observations.unconfirmed_landmark_id +=
        result.debug.update.observations_rejected.unconfirmed_landmark_id;
    _cumulative_diagnostics.rejected_observations.nonfinite_measurement +=
        result.debug.update.observations_rejected.nonfinite_measurement;
    _cumulative_diagnostics.rejected_observations.outside_measurement_range +=
        result.debug.update.observations_rejected.outside_measurement_range;
    _cumulative_diagnostics.rejected_observations.duplicate_landmark_id +=
        result.debug.update.observations_rejected.duplicate_landmark_id;

    _next_pose_index++;
    _previous_pose_key = current_pose_key;
    _previous_recorded_pose_odom_from_base = recorded_pose_odom_from_base;
    _previous_timestamp_ns = frame.timestamp_ns;

    result.debug.update_success = true;
    result.debug.core_failed = false;
    result.debug.cumulative = _cumulative_diagnostics;
    result.debug.message = "Success";

    return result;
}

IncrementalGraphSlamSnapshot IncrementalGraphSlam::snapshot() const
{
    IncrementalGraphSlamSnapshot snapshot;

    if (_failed)
    {
        snapshot.success = false;
        snapshot.message = "Incremental GraphSLAM core is in failed state";
        snapshot.totals = _cumulative_diagnostics;
        return snapshot;
    }

    if (_pose_metadata.empty())
    {
        snapshot.success = true;
        snapshot.message = "Graph is empty";
        snapshot.totals = _cumulative_diagnostics;
        return snapshot;
    }

    gtsam::Values current_estimates;
    try
    {
        current_estimates = _isam->calculateBestEstimate();
    }
    catch (const std::exception& e)
    {
        snapshot.success = false;
        snapshot.message =
            std::string(
                "Snapshot iSAM2 calculateBestEstimate threw exception: ") +
            e.what();
        snapshot.totals = _cumulative_diagnostics;
        return snapshot;
    }

    snapshot.poses.reserve(_pose_metadata.size());
    snapshot.landmarks.reserve(_initial_landmark_positions_map.size());

    try
    {
        for (const PoseMetadata& metadata : _pose_metadata)
        {
            const gtsam::Key key =
                gtsam::symbol_shorthand::X(metadata.pose_index);

            const gtsam::Pose2 optimized =
                current_estimates.at<gtsam::Pose2>(key);

            const PoseEstimate pose_est{
                metadata.pose_index,
                metadata.frame_index,
                metadata.timestamp_ns,
                metadata.initial_pose_map_from_base,
                metadata.recorded_pose_odom_from_base,
                to_transform_pose(optimized)};

            snapshot.poses.push_back(pose_est);
        }

        for (const auto& [mapper_id, initial_landmark_position_map] :
             _initial_landmark_positions_map)
        {
            const gtsam::Key key = gtsam::symbol_shorthand::L(mapper_id);

            const gtsam::Point2 optimized =
                current_estimates.at<gtsam::Point2>(key);

            const LandmarkEstimate landmark_est{
                mapper_id, to_transform_point(initial_landmark_position_map),
                to_transform_point(optimized)};

            snapshot.landmarks.push_back(landmark_est);
        }
    }
    catch (const std::exception& e)
    {
        snapshot.success = false;
        snapshot.message =
            std::string(
                "Snapshot iSAM2 estimate extraction threw exception: ") +
            e.what();
        snapshot.totals = _cumulative_diagnostics;
        return snapshot;
    }

    snapshot.latest_pose_map_from_odom =
        snapshot.poses.back().optimized_pose_map_from_base.compose(
            snapshot.poses.back().recorded_pose_odom_from_base.inverse());

    snapshot.success = true;
    snapshot.message = "Success";
    snapshot.totals = _cumulative_diagnostics;

    return snapshot;
}

bool IncrementalGraphSlam::_params_are_ok()
{
    auto is_positive_and_finite = [](double val) noexcept
    { return std::isfinite(val) && val > 0.0; };

    if (!is_positive_and_finite(
            _incremental_graph_slam_params.prior_pose_noise.x_std_m) ||
        !is_positive_and_finite(
            _incremental_graph_slam_params.prior_pose_noise.y_std_m) ||
        !is_positive_and_finite(
            _incremental_graph_slam_params.prior_pose_noise.yaw_std_rad))
    {
        return false;
    }

    if (!is_positive_and_finite(
            _incremental_graph_slam_params.odom_pose_noise.x_std_m) ||
        !is_positive_and_finite(
            _incremental_graph_slam_params.odom_pose_noise.y_std_m) ||
        !is_positive_and_finite(
            _incremental_graph_slam_params.odom_pose_noise.yaw_std_rad))
    {
        return false;
    }

    if (!is_positive_and_finite(
            _incremental_graph_slam_params.landmark_measurement_noise
                .bearing_std_rad) ||
        !is_positive_and_finite(_incremental_graph_slam_params
                                    .landmark_measurement_noise.range_std_m))
    {
        return false;
    }

    const double min_r = _incremental_graph_slam_params.measurement_range.min_m;
    const double max_r = _incremental_graph_slam_params.measurement_range.max_m;

    if (!std::isfinite(min_r) || !std::isfinite(max_r))
    {
        return false;
    }

    if (min_r <= 0.0 || max_r <= min_r)
    {
        return false;
    }

    if (!std::isfinite(
            _incremental_graph_slam_params.isam2.relinearization_threshold) ||
        _incremental_graph_slam_params.isam2.relinearization_threshold <= 0)
    {
        return false;
    }

    if (!std::isfinite(
            _incremental_graph_slam_params.isam2.relinearization_skip) ||
        _incremental_graph_slam_params.isam2.relinearization_skip <= 0)
    {
        return false;
    }

    return true;
}
bool IncrementalGraphSlam::_frame_is_ok(const LandmarkFrame& frame)
{
    if (frame.timestamp_ns < 0)
    {
        return false;
    }

    if (_previous_timestamp_ns && frame.timestamp_ns <= *_previous_timestamp_ns)
    {
        return false;
    }

    if (!std::isfinite(frame.recorded_pose_odom_from_base.x_m) ||
        !std::isfinite(frame.recorded_pose_odom_from_base.y_m) ||
        !std::isfinite(frame.recorded_pose_odom_from_base.yaw_rad))
    {
        return false;
    }

    return true;
}

}  // namespace slam::backend
