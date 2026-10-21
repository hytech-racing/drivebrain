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

  // register callback for tuning parameters for controllers
  auto& foxglove = FoxgloveServer::instance();
  _parameter_connection = foxglove.register_param_callback(
      [this](const std::unordered_map<std::string, DBParam>& params) {
        std::lock_guard lock(_tuning_mutex);
        auto update = [&params](const char* name, float& value) {
          if (auto it = params.find(name); it != params.end()) {
            if (auto param = std::get_if<float>(&it->second); param && *param >= 0.0f) value = *param;
          }
        };
        update("purepursuitcontroller/lookahead_distance", _tuning.lookahead_distance);
        update("velocityplanner/max_velocity", _tuning.max_velocity);
        update("velocityplanner/mu", _tuning.mu);
      }, {"purepursuitcontroller/lookahead_distance", "velocityplanner/max_velocity", "velocityplanner/mu"});
  _slam.start();
  _thread = std::thread(&Autonomy::_run, this);
  spdlog::info("Autonomy stack started");
}

void Autonomy::stop() {
  if (!_running.exchange(false)) {
    return;
  }
  _parameter_connection.disconnect();
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
  Tuning tuning;
  {
    std::lock_guard lock(_tuning_mutex);
    tuning = _tuning;
  }
  _lateral_controller.setLookaheadDistance(tuning.lookahead_distance);
  _longitudinal_controller.setMaxCarVelocity(tuning.max_velocity);
  _longitudinal_controller.setMu(tuning.mu);

  static uint8_t prescale_counter = 0;
  auto dv = StateTracker::instance().dv_state();
  ControllerOutput out = {};
  
  // if (++prescale_counter < PRESCALE_COUNTER) {
  //   return out;
  // }
  // prescale_counter = 0;

  
  // run control loops
  if (dv.path && !dv.path->empty()) {
    float curvature;
    std::optional<float> steering_angle = _lateral_controller.step_controller(vehicle_state, *dv.path, curvature); // pure pursuit
    if (steering_angle.has_value()) {
      out.desired_steering_deg = steering_angle.value();
      _longitudinal_controller.setPath(*dv.path);
      out.out = _longitudinal_controller.step_controller(vehicle_state, curvature); // velocity controller
    } else {
      spdlog::error("Pure Pursuit controller failed to compute steering angle");
      out.out = std::monostate{};
    }
  } else {
    out.out = std::monostate{};
  }

  //out.out = _longitudinal_controller.step_controller(vehicle_state); // velocity controller


  // fetch pure pursuit logs
  // auto msg = _lateral_controller.getLoggingData();
  
  // #if HOOTL_ENABLED
  //   // std::cout << msg.
  //   comms::SimComms::instance().send_message(msg);
  // #endif

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
      render_path(path, "planned_path", "map"); //draws a line between path points 
      StateTracker::instance().set_dv_path(
          std::make_shared<const std::vector<xy_vec<float>>>(std::move(path)));
    }

    std::this_thread::sleep_until(next_tick);
  }
}

}
