#include "autonomy.hpp"
#if HOOTL_ENABLED
#include "SimComms.hpp"
#endif
#include <memory>
#include <stdexcept>
#include <variant>
#include <vector>

#include <spdlog/spdlog.h>

#include <Telemetry.hpp>
#include <StateTracker.hpp>
#include <PathPlanner.hpp>

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

  if (!_lateral_controller.init()) {
    _running = false;
    throw std::runtime_error("Failed to initialize pure pursuit controller");
  }

  if (!_longitudinal_controller.init()) {
    _running = false;
    throw std::runtime_error("Failed to initialize velocity planner");
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
  auto dv = StateTracker::instance().dv_state();
  ControllerOutput out = {};
  
  // if (++prescale_counter < PRESCALE_COUNTER) {
  //   return out;
  // }
  // prescale_counter = 0;

  
  // run control loops
  if (dv.path && !dv.path->empty()) {
    SpeedControlOut speed_out{};
    speed_out.mcu_recv_millis = vehicle_state.prev_MCU_recv_millis;
    float curvature;

    // safely break to 0ms if steering fails
    speed_out.desired_rpms = {0.0f, 0.0f, 0.0f, 0.0f};
    speed_out.torque_lim_nm = {2.5f, 2.5f, 2.5f, 2.5f}; // 1/4 of max regen torque 
    out.out = speed_out;
    out.desired_steering_deg = 0.0f;


    std::optional<float> steering_angle = _lateral_controller.step_controller(vehicle_state, *dv.path, curvature); // pure pursuit
    log(std::make_shared<dv_msgs::PurePursuitDebug>(_lateral_controller.debug()), "control/pure_pursuit_debug");
    if (steering_angle.has_value()) {
      out.desired_steering_deg = steering_angle.value();
      _longitudinal_controller.set_path(*dv.path);
      out.out = _longitudinal_controller.step_controller(vehicle_state, curvature); // velocity controller
      log(std::make_shared<dv_msgs::VelocityPlannerDebug>(_longitudinal_controller.debug()), "planning/velocity_debug");
    } else {
      spdlog::error("Pure Pursuit controller failed to compute steering angle");
    }
  } 

  //out.out = _longitudinal_controller.step_controller(vehicle_state); // velocity controller


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

      std::shared_ptr<dv_msgs::Cones> cones = perception::filter_cloud(*last_scan);

      std::cout << "Filtered cones: " << cones->cones_size() << std::endl;

      core::render_cones(cones, "filtered_cones", last_scan->frame_id());

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
