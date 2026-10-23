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
#include <SimComms.hpp>

namespace core {

Autonomy::~Autonomy() {
  stop();
}

void Autonomy::start() {
  if (_running.exchange(true)) {
    return;
  }
  _slam = std::make_unique<slam::Slam>();

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
  out.out = std::monostate{};
  return out;
}

void Autonomy::_run() {
  std::shared_ptr<const foxglove::PointCloud> last_scan;

  // Plan a path through the cones (map frame) and hand it to the controllers
  auto plan = [](const dv_msgs::Cones& cones) {
    auto path = planning::plan_path(cones);
    render_path(path, "planned_path", "map");
    StateTracker::instance().set_dv_path(std::make_shared<const std::vector<xyz_vec<float>>>(std::move(path)));
  };

  while (_running) {
    const auto scan = StateTracker::instance().wait_for_scan(last_scan, SCAN_WAIT);
    if (!scan) continue;
    last_scan = scan;
    const uint64_t scan_ns = google::protobuf::util::TimeUtil::TimestampToNanoseconds(last_scan->timestamp());

    // With a sim (HOOTL or the sim bridge), plan from its ground truth cones, which are in the same frame
    // as the sim pose that pure pursuit tracks, and skip perception and SLAM
    if (comms::SimComms::is_created()) {
      if (const auto ground_truth_cones = StateTracker::instance().dv_state().cone_observations) {
        plan(*ground_truth_cones);
      }
      continue;
    }

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
    const std::shared_ptr<dv_msgs::Cones> cones = perception::filter_cloud(*cloud);
    render_cones(cones, "filtered_cones", "os_sensor");

    // Perform SLAM update with the filtered cones
    const auto map = _slam->update(*cones, scan_ns, *pose, T_base_lidar);
    if (!map) continue;

    // Update the state tracker with the new cone observations and plan a path
    StateTracker::instance().set_cone_observations(map);
    plan(*map);
  }
}

}
