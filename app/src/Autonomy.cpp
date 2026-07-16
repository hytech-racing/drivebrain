#include "Autonomy.hpp"

#include <memory>
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

// TODO: Pure pursuit impl should be invoked here
ControllerOutput Autonomy::command(const VehicleState& vehicle_state) {
  ControllerOutput out;
  out.out = std::monostate{};
  return out;
}

void Autonomy::_run() {
  auto next_tick = std::chrono::steady_clock::now();
  std::shared_ptr<const foxglove::PointCloud> last_scan;

  while (_running) {
    next_tick += _period;

    auto dv = StateTracker::instance().dv_state();

    if (dv.lidar_is_valid && dv.lidar_cloud != last_scan) {
      last_scan = dv.lidar_cloud;

      // TODO: cone classifier needs to be invoked here
      auto observed_cones = std::make_shared<dv_msgs::Cones>();
      StateTracker::instance().set_cone_observations(observed_cones);

      auto path = planning::plan_path(*observed_cones);
      render_path(path, "planned_path", "map");
      StateTracker::instance().set_dv_path(
          std::make_shared<const std::vector<xyz_vec<float>>>(std::move(path)));
    }

    std::this_thread::sleep_until(next_tick);
  }
}

}
