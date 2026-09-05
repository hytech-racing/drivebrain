#include "autonomy.hpp"
#include "SimComms.hpp"

#include <memory>
#include <variant>
#include <vector>

#include <spdlog/spdlog.h>

#include <Telemetry.hpp>
#include <StateTracker.hpp>
#include <PathPlanner.hpp>
#include <autonomy_msgs.pb.h>

// todo: abstract this out to a config file or something, but for now just hardcode it to 25Hz
#define LOOP_TIME 0.004 
#define CONTROLLER_LOOP_TIME 0.01
#define PRESCALE_COUNTER ((int)(CONTROLLER_LOOP_TIME / LOOP_TIME))

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
  return true;
  auto dv = StateTracker::instance().dv_state();
  return dv.lidar_is_valid && dv.path && !dv.path->empty();
}


ControllerOutput Autonomy::command(const VehicleState& vehicle_state) {

  static uint8_t prescale_counter = 0;
  ControllerOutput out;
  if (++prescale_counter < PRESCALE_COUNTER) {
    return out;
  }
  prescale_counter = 0;
  // run control loop 
  out =_controller.step_controller(vehicle_state);
  // fetch logs
  auto msg = _controller.getLoggingData();
  
#if HOOTL_ENABLED
  // std::cout << msg.
  comms::SimComms::instance().send_message(msg);
#endif
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
