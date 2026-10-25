#include "autonomy.hpp"

#include <cmath>
#include <memory>
#include <variant>
#include <vector>

#include <google/protobuf/util/time_util.h>
#include <spdlog/spdlog.h>

#include <Telemetry.hpp>
#include <LidarFilter.hpp>
#include <PointCloudDeskew.hpp>
#include <PathPlanner.hpp>

namespace core {

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
    auto path = planning::plan_path(*map);
    render_path(path, "planned_path", "map", {1.0, 0.0, 1.0, 1.0}); // magenta, apart from the green trails
    render_command(StateTracker::instance().vehicle_state().first, control::PurePursuitController::WHEELBASE_M, "steering_command");

    StateTracker::instance().set_dv_path(std::make_shared<const std::vector<xyz_vec<float>>>(std::move(path))); // Used by the pure pursuit controller
  }
}

}
