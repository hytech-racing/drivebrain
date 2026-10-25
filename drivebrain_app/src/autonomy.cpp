#include "autonomy.hpp"

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

    // Filter the deskewed point cloud for cones
    std::shared_ptr<dv_msgs::Cones> cones = perception::filter_cloud(*cloud);
    if (_cone_classifier) {
      cones = _cone_classifier->classify(cones);
    }
    render_cones(cones, "filtered_cones", "os_sensor");

    // Perform SLAM update with the filtered cones
    const auto map = _slam->update(*cones, scan_ns, *pose, T_base_lidar);
    if (!map) continue;

    // Update the state tracker with the new cone observations and plan a path
    StateTracker::instance().set_cone_observations(map);
    auto path = planning::plan_path(*map);
    render_path(path, "planned_path", "map");

    StateTracker::instance().set_dv_path(std::make_shared<const std::vector<xyz_vec<float>>>(std::move(path))); // Used by the pure pursuit controller
  }
}

}
