#include "GraphSlam.hpp"

#include <cmath>
#include <exception>
#include <string>

#include <spdlog/spdlog.h>

#include <FoxgloveServer.hpp>
#include <Geometry.hpp>
#include <LidarFilter.hpp>
#include <Telemetry.hpp>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/sam/BearingRangeFactor.h>
#include <gtsam/slam/BetweenFactor.h>

namespace slam {

namespace {

using gtsam::symbol_shorthand::L;
using gtsam::symbol_shorthand::X;

gtsam::Pose2 gtsam_pose(const Eigen::Isometry2d& pose) {
    return {pose.translation().x(), pose.translation().y(), core::geometry::yaw(pose)};
}

Eigen::Isometry2d eigen_pose(const gtsam::Pose2& pose) {
    return core::geometry::make_pose2d(pose.x(), pose.y(), pose.theta());
}

}  // namespace

GraphSlam::GraphSlam()
    : _isam([] {
          gtsam::ISAM2Params p;
          p.evaluateNonlinearError = true;
          return p;
      }()) {
    const double odom_xy = core::FoxgloveServer::instance().get_param<float>("Slam/odom_noise_xy_m").value();
    const double odom_yaw = core::FoxgloveServer::instance().get_param<float>("Slam/odom_noise_yaw_rad").value();
    _odom_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(odom_xy, odom_xy, odom_yaw));
    const double bearing_std = core::FoxgloveServer::instance().get_param<float>("Slam/bearing_std_rad").value();
    const double range_std = core::FoxgloveServer::instance().get_param<float>("Slam/range_std_m").value();
    _bearing_range_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(bearing_std, range_std));
}

std::optional<MapState> GraphSlam::update(const ObservationFrame& frame) {
    _debug.set_timestamp_ns(frame.timestamp_ns);
    _debug.set_frame_index(_pose_count);
    _debug.set_core_failed(_failed);
    _debug.clear_isam2();
    dv_msgs::IncrementalGraphSlamUpdateCounts& update = *_debug.mutable_update();
    update.Clear();
    update.set_observations_received(frame.observations.size());
    dv_msgs::SlamObservationRejectionCounts& rejected = *update.mutable_observations_rejected();
    if (_failed) {
        return std::nullopt;
    }

    try {
        gtsam::NonlinearFactorGraph& factors = _pending_factors;
        gtsam::Values& values = _pending_values;

        const gtsam::Pose2 pose_odom_from_base = gtsam_pose(frame.pose_odom_from_base);
        const gtsam::Key pose_key = X(_pose_count);

        gtsam::Pose2 pose_guess;
        if (_pose_count == 0) {
            factors.addPrior(pose_key, gtsam::Pose2(), _prior_noise);
        } else {
            const gtsam::Pose2 odom_delta =
                _previous_pose_odom_from_base.between(pose_odom_from_base);
            factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose2>>(
                X(_pose_count - 1), pose_key, odom_delta, _odom_noise);
            pose_guess = _previous_pose_guess.compose(odom_delta);
        }
        values.insert(pose_key, pose_guess);

        std::set<std::uint64_t> seen_ids;
        std::set<std::uint64_t> new_ids;
        for (const LandmarkObservation& observation : frame.observations) {
            const gtsam::Point2& measurement = observation.measurement_base_m;
            if (!measurement.allFinite()) {
                rejected.set_nonfinite_measurement(rejected.nonfinite_measurement() + 1);
                continue;
            }

            const double range = std::hypot(measurement.x(), measurement.y());
            if (!seen_ids.insert(observation.landmark_id).second) {
                rejected.set_duplicate_landmark_id(rejected.duplicate_landmark_id() + 1);
                continue;
            }

            // A new landmark must not exist yet; any other association must reference one that does
            const bool is_new = observation.status == LandmarkStatus::New;
            const bool known = _landmark_ids.count(observation.landmark_id) > 0;
            if (is_new == known) {
                rejected.set_unconfirmed_landmark_id(rejected.unconfirmed_landmark_id() + 1);
                continue;
            }

            const gtsam::Key landmark_key = L(observation.landmark_id);
            if (is_new) {
                values.insert(landmark_key, pose_guess.transformFrom(measurement));
                new_ids.insert(observation.landmark_id);
            }

            factors.emplace_shared<gtsam::BearingRangeFactor<gtsam::Pose2, gtsam::Point2>>(
                pose_key, landmark_key, gtsam::Rot2(std::atan2(measurement.y(), measurement.x())),
                range, _bearing_range_noise);
            update.set_observations_admitted(update.observations_admitted() + 1);
        }

        _landmark_ids.insert(new_ids.begin(), new_ids.end());
        _previous_pose_odom_from_base = pose_odom_from_base;
        _previous_pose_guess = pose_guess;
        _pose_count++;

        update.set_new_landmarks_added(new_ids.size());
        dv_msgs::IncrementalGraphSlamTotals& totals = *_debug.mutable_cumulative();
        totals.set_pose_count(_pose_count);
        totals.set_landmark_count(_landmark_ids.size());
        totals.set_measurement_factor_count(totals.measurement_factor_count() + update.observations_admitted());

        if (++_frames_since_solve < SOLVE_PER_FRAME) {
            return std::nullopt;
        }
        _frames_since_solve = 0;

        const gtsam::ISAM2Result result = _isam.update(factors, values);
        factors.resize(0);
        values.clear();
        _previous_pose_guess = _isam.calculateEstimate<gtsam::Pose2>(pose_key);

        const gtsam::Pose2 pose_map_from_odom = _previous_pose_guess * pose_odom_from_base.inverse();

        dv_msgs::Isam2UpdateDiagnostics& isam2 = *_debug.mutable_isam2();
        isam2.set_has_error_before_update(result.errorBefore.has_value());
        isam2.set_error_before_update(result.errorBefore.value_or(0.0));
        isam2.set_has_error_after_update(result.errorAfter.has_value());
        isam2.set_error_after_update(result.errorAfter.value_or(0.0));
        isam2.set_variables_relinearized(result.variablesRelinearized);
        isam2.set_variables_reeliminated(result.variablesReeliminated);
        _debug.set_pose_map_from_odom_x_m(pose_map_from_odom.x());
        _debug.set_pose_map_from_odom_y_m(pose_map_from_odom.y());
        _debug.set_pose_map_from_odom_yaw_rad(pose_map_from_odom.theta());

        MapState map;
        map.sequence = _next_sequence++;
        map.timestamp_ns = frame.timestamp_ns;
        map.pose_map_from_odom = eigen_pose(pose_map_from_odom);
        for (const std::uint64_t id : _landmark_ids) {
            map.landmarks.push_back({id, _isam.calculateEstimate<gtsam::Point2>(L(id))});
        }
        return map;
    } catch (const std::exception& error) {
        _failed = true;
        _debug.set_core_failed(true);
        _debug.set_message(error.what());
        spdlog::error("GraphSlam: rejected a frame, mapping stopped: {}", error.what());
        return std::nullopt;
    }
}

std::map<std::size_t, Eigen::Isometry2d> GraphSlam::optimized_poses() const {
    std::map<std::size_t, Eigen::Isometry2d> poses;
    for (const auto& [key, pose] : _isam.calculateEstimate().extract<gtsam::Pose2>()) {
        poses.emplace(gtsam::Symbol(key).index(), eigen_pose(pose));
    }
    return poses;
}

std::shared_ptr<dv_msgs::Cones> Slam::update(const dv_msgs::Cones& cones, const std::uint64_t scan_ns,
                                             const Eigen::Isometry2d& pose_odom_from_base,
                                             const Eigen::Isometry3d& T_base_lidar) {
    // detections to base_link, confidence from the range to the lidar
    ConeFrame frame{scan_ns, pose_odom_from_base, {}};
    for (const auto& cone : cones.cones()) {
        const Eigen::Vector3d p_lidar(cone.position().x(), cone.position().y(), cone.position().z());
        const Eigen::Vector3d p_base = T_base_lidar * p_lidar;
        const float range_m = static_cast<float>(std::hypot(p_lidar.x(), p_lidar.y()));
        frame.detections.push_back(
            {p_base.head<2>(), perception::detection_confidence(range_m), cone.color(), 0.0});
    }

    const std::optional<ObservationFrame> observations = frontend.process_frame(frame);
    if (observations) { // send candidates to backend
        if (const std::optional<MapState> solved = backend.update(*observations)) {
            frontend.set_map(*solved);
            map_state = *solved;
            core::publish_transform("map", "odom", frame.timestamp_ns,
                                    core::geometry::to_pose3d(solved->pose_map_from_odom));
        }
    }
    core::log(std::make_shared<dv_msgs::SlamFrontendDebug>(frontend.debug()), "slam/frontend_debug");
    core::log(std::make_shared<dv_msgs::IncrementalGraphSlamDebug>(backend.debug()), "slam/backend_debug");

    const std::shared_ptr<dv_msgs::Cones> map = frontend.cone_map(frame.timestamp_ns);
    if (map && map_state) {
        render(frame.timestamp_ns, observations, map);
    }
    return map;
}

void Slam::render(const std::uint64_t scan_ns, const std::optional<ObservationFrame>& observations,
                  const std::shared_ptr<dv_msgs::Cones>& map) {
    core::render_cones(map, "slam_landmarks");

    // this frame's observation factors: car pose -> observed landmark
    if (observations) {
        const Eigen::Isometry2d pose_map_from_base = map_state->pose_map_from_odom * observations->pose_odom_from_base;
        const Eigen::Vector2d car = pose_map_from_base.translation();
        std::vector<core::xyz_vec<float>> factors;
        for (const LandmarkObservation& observation : observations->observations) {
            const Eigen::Vector2d measured = pose_map_from_base * observation.measurement_base_m;
            factors.push_back({static_cast<float>(car.x()), static_cast<float>(car.y()), 0.1f});
            factors.push_back({static_cast<float>(measured.x()), static_cast<float>(measured.y()), 0.1f});
        }
        core::render_segments(factors, "slam_factors");
    }

    // Redrawn once a second
    if (scan_ns - last_graph_render_ns >= 1'000'000'000ULL) {
        last_graph_render_ns = scan_ns;
        std::vector<core::xyz_vec<float>> chain;
        for (const auto& [index, pose] : backend.optimized_poses()) {
            chain.push_back({static_cast<float>(pose.translation().x()), static_cast<float>(pose.translation().y()), 0.05f});
        }
        core::render_path(chain, "slam_graph");
    }
}

}  // namespace slam
