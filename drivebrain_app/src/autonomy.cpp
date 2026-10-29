#include "autonomy.hpp"

#include <algorithm>
#include <cmath>
#include <csignal>
#include <memory>
#include <variant>
#include <vector>

#include <google/protobuf/util/time_util.h>
#include <spdlog/spdlog.h>

#include <FoxgloveServer.hpp>
#include <Telemetry.hpp>
#include <LidarFilter.hpp>
#include <PointCloudDeskew.hpp>
#include <PathPlanner.hpp>

namespace core {
namespace {
// Distance from the sim's car reference point (CG) to its lidar, along the car. Matches MOUNT_FWD in xcel sim/scene.py
constexpr double SIM_LIDAR_FORWARD_M = 0.75;
constexpr double SIM_COLOR_MATCH_M = 0.6;

// HOOTL only: the sim has no cameras, so no cone ever gets a colour. Copy the colour of the nearest true cone instead.
// Does nothing on the real car (no ground truth is ever set). Colours already set by the camera classifier are kept.
void color_cones_from_sim_ground_truth(dv_msgs::Cones& cones) {
  const auto dv = StateTracker::instance().dv_state();
  if (!dv.ground_truth_cones || !dv.has_ground_truth_pose) return;

  const double c = std::cos(dv.ground_truth_yaw), s = std::sin(dv.ground_truth_yaw);
  const double ox = dv.ground_truth_x + SIM_LIDAR_FORWARD_M * c, oy = dv.ground_truth_y + SIM_LIDAR_FORWARD_M * s;
  for (auto& cone : *cones.mutable_cones()) {
    if (cone.color() != dv_msgs::Cones_ConeColor_UNKNOWN) continue;
    // sensor frame (car-forward along -y, car-left along +x) -> car frame -> sim world frame
    const double car_fwd = -cone.position().y(), car_left = cone.position().x();
    const double wx = ox + c * car_fwd - s * car_left;
    const double wy = oy + s * car_fwd + c * car_left;
    double best = SIM_COLOR_MATCH_M;
    for (const auto& truth : dv.ground_truth_cones->cones()) {
      const double d = std::hypot(truth.position().x() - wx, truth.position().y() - wy);
      if (d < best) {
        best = d;
        cone.set_color(truth.color());
        cone.set_color_confidence(1.0f);  // SLAM's colour voting ignores zero-confidence colours
      }
    }
  }
}
}  // namespace

Autonomy::~Autonomy() {
  stop();
}

void Autonomy::start() {
  if (_running.exchange(true)) {
    return;
  }
  _slam = std::make_unique<slam::Slam>();
  _controller = std::make_unique<control::PurePursuitController>();
  _cone_classifier = std::make_unique<perception::ConeClassifier>("intrinsics");

  auto& params = FoxgloveServer::instance();
  planning::RacingLinePlanner::Config racing_line;
  racing_line.enabled = params.get_param<bool>("racing_line/enabled").value_or(false);
  if (const auto v = params.get_param<float>("racing_line/car_half_width_m")) racing_line.line.car_half_width_m = *v;
  if (const auto v = params.get_param<float>("racing_line/cone_radius_m")) racing_line.line.cone_radius_m = *v;
  if (const auto v = params.get_param<float>("racing_line/clearance_m")) racing_line.line.clearance_m = *v;
  if (const auto v = params.get_param<float>("racing_line/gate_spacing_m")) racing_line.line.gate_spacing_m = *v;
  if (const auto v = params.get_param<float>("racing_line/max_cone_gap_m")) racing_line.line.max_cone_gap_m = *v;
  if (const auto v = params.get_param<float>("racing_line/replan_period_s")) racing_line.replan_period_s = *v;
  if (const auto v = params.get_param<int>("racing_line/midline_laps_after_closure")) racing_line.midline_laps_after_closure = std::max(*v, 0);
  if (const auto v = params.get_param<int>("racing_line/racing_laps")) racing_line.racing_laps = std::max(*v, 1);
  if (const auto v = params.get_param<bool>("racing_line/end_run_when_done")) racing_line.end_run_when_done = *v;
  if (const auto v = params.get_param<int>("racing_line/window_points")) racing_line.window_points = static_cast<std::size_t>(std::max(*v, 2));
  // The speed held on the discovery lap (and throughout when the speed profile is off) comes from the config too
  if (const auto v = params.get_param<float>("speed_profile/discovery_speed_mps"); v && *v > 0.0f) {
    _controller->set_constant_speed(*v);
    spdlog::info("Discovery lap speed {} m/s", *v);
  } else {
    spdlog::error("speed_profile/discovery_speed_mps is missing or not positive: the car will not move until it is set");
  }
  // Every value comes from the "speed_profile" config block; if any is missing or unusable the car keeps a constant speed
  std::optional<control::SpeedProfileParams> speed_profile;
  if (params.get_param<bool>("speed_profile/enabled").value_or(false)) {
    const auto max_speed = params.get_param<float>("speed_profile/max_speed_mps");
    const auto min_speed = params.get_param<float>("speed_profile/min_speed_mps");
    const auto lat_accel = params.get_param<float>("speed_profile/max_lat_accel_mps2");
    const auto brake = params.get_param<float>("speed_profile/max_brake_mps2");
    const auto accel = params.get_param<float>("speed_profile/max_accel_mps2");
    if (max_speed && min_speed && lat_accel && brake && accel && *min_speed > 0.0f && *max_speed >= *min_speed && *lat_accel > 0.0f && *brake > 0.0f &&
        *accel > 0.0f) {
      speed_profile = control::SpeedProfileParams{*max_speed, *min_speed, *lat_accel, *brake, *accel};
    } else {
      spdlog::warn("speed_profile/enabled is true but its values are missing or invalid (need max_speed_mps >= min_speed_mps > 0, max_lat_accel_mps2 > 0, max_brake_mps2 > 0, max_accel_mps2 > 0); using a constant speed");
    }
  }
  _controller->set_speed_profile(speed_profile);
  spdlog::info("Speed profile {}", speed_profile ? "enabled" : "disabled (constant speed)");
  _racing_line = planning::RacingLinePlanner(racing_line);
  spdlog::info("Racing line planner {}: 1 discovery lap, {} midline, {} racing line; {}", racing_line.enabled ? "enabled" : "disabled",
               racing_line.midline_laps_after_closure, racing_line.racing_laps,
               racing_line.end_run_when_done ? "the run ends after the last lap" : "the run does not end by itself (racing_line/end_run_when_done is false or missing)");
  _thread = std::thread(&Autonomy::_run, this);
  spdlog::info("Autonomy stack started");
}

void Autonomy::stop() {
  if (!_running.exchange(false)) {
    return;
  }
  if (_thread.joinable()) {
    _thread.join();
  }
  spdlog::info("Autonomy stack stopped");
}

bool Autonomy::is_valid() {
  auto dv = StateTracker::instance().dv_state();
  return dv.lidar_is_valid && dv.path && !dv.path->empty();
}

ControllerOutput Autonomy::command(const VehicleState& vehicle_state) {
  ControllerOutput out;
  if (_controller) {
    out = _controller->step(vehicle_state);
  } else {
    out.out = std::monostate{};
  }
  return out;
}

void Autonomy::_run() {
  std::shared_ptr<const foxglove::PointCloud> last_scan;

  while (_running) {
    const auto scan = StateTracker::instance().wait_for_scan(last_scan, SCAN_WAIT);
    if (!scan) continue;
    last_scan = scan;
    const uint64_t scan_ns = google::protobuf::util::TimeUtil::TimestampToNanoseconds(last_scan->timestamp());

    // Get odometry pose for the scan
    const auto pose = StateTracker::instance().odom_pose(scan_ns, ODOM_TIMEOUT);
    if (!pose) {
      spdlog::warn("Autonomy: no odom pose for scan at {} ns, skipping SLAM", scan_ns);
      continue;
    }

    // Deskew the point cloud
    const Eigen::Isometry3d& T_base_lidar = StateTracker::instance().T_base_lidar();
    publish_transform("base_link", "os_sensor", scan_ns, T_base_lidar);
    const auto cloud = perception::deskew_cloud(*last_scan, StateTracker::instance().odom_twist(), T_base_lidar);
    log_foxglove_only(cloud, "perception/deskewed_points");

    std::shared_ptr<dv_msgs::Cones> cones = perception::filter_cloud(*cloud);
    cones = _classifier.classify(cones, StateTracker::instance().dv_state().camera_detections, last_scan->frame_id());
    color_cones_from_sim_ground_truth(*cones);
    render_cones(cones, "filtered_cones", "os_sensor");

    // Perform SLAM update with the filtered cones
    const auto map = _slam->update(*cones, scan_ns, *pose, T_base_lidar);
    if (!map) continue;

    if (_slam->map_state) {
      const Eigen::Isometry2d pose_map = _slam->map_state->pose_map_from_odom * *pose;
      const double yaw = std::atan2(pose_map.linear()(1, 0), pose_map.linear()(0, 0));
      auto map_pose = std::make_shared<hytech_msgs::pose>();
      map_pose->mutable_position()->set_x(pose_map.translation().x());
      map_pose->mutable_position()->set_y(pose_map.translation().y());
      map_pose->mutable_orientation()->set_w(std::cos(yaw / 2));
      map_pose->mutable_orientation()->set_z(std::sin(yaw / 2));
      log(map_pose, "slam/vehicle_pose");                                  // MCAP + Foxglove
      render_pose(map_pose, "vehicle_pose_map");                           // box in the 3D panel, map frame
      StateTracker::instance().handle_receive_protobuf_message(map_pose);  // planner + pure pursuit
      StateTracker::instance().set_pose_map_from_odom(_slam->map_state->pose_map_from_odom);
    }

    // Update the state tracker with the new cone observations and plan a path
    StateTracker::instance().set_cone_observations(map);
    const auto midline = planning::plan_path(*map);
    render_path(midline, "midline_path", "map", {1.0, 0.5, 0.0, 1.0}, "scene/midline_path"); // orange
    const auto racing_path = _racing_line.plan(*map, scan_ns);
    _controller->set_track_closed(_racing_line.track_closed());  // the speed profile starts once the discovery lap is over
    _controller->set_finished(_racing_line.finished());          // all laps done: stop the car
    if (_racing_line.finished() && !_end_requested) {
      // Once the car has stopped, shut drivebrain down the way Ctrl+C does, so the MCAP is closed and indexed
      if (std::fabs(StateTracker::instance().odom_twist().x()) < 0.2) {  // odometry speed: the VectorNav velocity is not populated in the sim
        _end_requested = true;
        spdlog::info("Run complete and the car has stopped: shutting down");
        std::raise(SIGINT);
      }
    }
    if (racing_path) render_path(*racing_path, "racing_line_path", "map", {0.2, 1.0, 0.1, 1.0}, "scene/racing_line_path"); // green
    const auto& path = racing_path ? *racing_path : midline;
    render_path(path, "planned_path", "map", {1.0, 0.0, 1.0, 1.0}, "scene/planned_path"); // magenta: what the controller follows, apart from the green trails

    StateTracker::instance().set_dv_path(std::make_shared<const std::vector<xyz_vec<float>>>(path)); // Used by the pure pursuit controller
  }
}

}
