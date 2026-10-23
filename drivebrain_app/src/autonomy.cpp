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

// TODO: Pure pursuit impl should be invoked here. Hardcoded for now to verify the
// command path from drivebrain through to the sim's vehicle dynamics.
ControllerOutput Autonomy::command(const VehicleState& vehicle_state) {
  // TorqueControlOut torque;
  // torque.desired_torques_nm = {2.0f, 2.0f, 2.0f, 2.0f};

  ControllerOutput out;
  out.out = std::monostate{};
  // out.out = torque;
  // out.desired_steering_deg = 10.0f;
  return out;
}

void Autonomy::_run() {
  auto next_tick = std::chrono::steady_clock::now();
  std::shared_ptr<const foxglove::PointCloud> last_scan;

  while (_running) {
    next_tick += PERIOD;
    std::this_thread::sleep_until(next_tick);

    auto dv = StateTracker::instance().dv_state();
    if (!dv.lidar_is_valid || dv.lidar_cloud == last_scan) continue;
    last_scan = dv.lidar_cloud;
    const uint64_t scan_ns = google::protobuf::util::TimeUtil::TimestampToNanoseconds(last_scan->timestamp());

    const auto pose = StateTracker::instance().odom_pose(scan_ns, ODOM_TIMEOUT);
    if (!pose) {
      spdlog::warn("Autonomy: no odom pose for scan at {} ns, skipping SLAM", scan_ns);
      continue;
    }

    const Eigen::Isometry3d& T_base_lidar = StateTracker::instance().T_base_lidar();
    publish_transform("base_link", "os_sensor", scan_ns, T_base_lidar);
    const auto cloud = perception::deskew_cloud(*last_scan, StateTracker::instance().odom_twist(), T_base_lidar);
    if (LOG_DESKEWED_CLOUD) log(cloud, "perception/deskewed_points");

    const std::shared_ptr<dv_msgs::Cones> cones = perception::filter_cloud(*cloud);
    render_cones(cones, "filtered_cones", "os_sensor");

    const auto map = _slam->update(*cones, scan_ns, *pose, T_base_lidar);
    if (!map) continue;

    StateTracker::instance().set_cone_observations(map);
    auto path = planning::plan_path(*map);
    StateTracker::instance().set_dv_path(std::make_shared<const std::vector<xyz_vec<float>>>(std::move(path)));
    // do controls!
  }
}

}
