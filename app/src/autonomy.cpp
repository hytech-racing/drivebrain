#include "autonomy.hpp"

#include <memory>
#include <variant>
#include <vector>

#include <spdlog/spdlog.h>

#include <Telemetry.hpp>
#include <StateTracker.hpp>
#include <PathPlanner.hpp>

namespace core {

Autonomy::~Autonomy() {
  stop();
}

void Autonomy::start() {
  if (_running.exchange(true)) {
    return;
  }
  _slam.start();
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
  _slam.stop();
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
    next_tick += _period;

    auto dv = StateTracker::instance().dv_state();

    if (dv.lidar_is_valid && dv.lidar_cloud != last_scan && dv.cone_observations) {
      last_scan = dv.lidar_cloud;

      // TODO: cone classifier needs to be invoked here
      auto path = planning::plan_path(*StateTracker::instance().dv_state().cone_observations);
      render_path(path, "planned_path", "lidar");
      StateTracker::instance().set_dv_path(
          std::make_shared<const std::vector<xyz_vec<float>>>(std::move(path)));
    }

    std::this_thread::sleep_until(next_tick);
  }
}

}
