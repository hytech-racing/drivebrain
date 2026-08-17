#include <StateTracker.hpp>
#include <atomic>
#include <chrono>
#include <mcap/writer.hpp>
#include <memory>
#include <fmt/chrono.h>
#include <spdlog/spdlog.h>
#include <filesystem>
#include <stdexcept>

#include "app.hpp"
#include "ETHRecvComms.hpp"
#include "FoxgloveServer.hpp"
#include "MCAPLogger.hpp"
#include "MatlabModelAddHelper.hpp"
#include "Telemetry.hpp"
#include "ControllerManager.hpp"
#include "DrivebrainControllerInterface.hpp"

#include "dv_msgs.pb.h"
#include "hytech_msgs.pb.h"

std::atomic<bool> running{true};

void sig_handler(int signal) {
    if(signal == SIGINT) {
      spdlog::warn("Interrupted, stopping Drivebrain app");
      running = false;
    }
}

DrivebrainApp::DrivebrainApp(const std::string& json_param_path, const std::string& dbc_path)
  : _json_params_path(json_param_path)
  , _dbc_path(dbc_path) {}

DrivebrainApp::~DrivebrainApp() {
  running = false;

  core::MCAPLogger::instance().destroy();
  core::FoxgloveServer::instance().destroy();
  spdlog::info("Logging singletons safely destroyed");
}

void DrivebrainApp::run() {
  std::signal(SIGINT, sig_handler);

  core::MCAPLogger::create("recordings/", mcap::McapWriterOptions(""), _json_params_path);
  core::FoxgloveServer::create(_json_params_path);
  core::StateTracker::create();

  core::MCAPLogger::instance().open_new_mcap();
  core::MCAPLogger::instance().init_logging();

  spdlog::info("Constructed logging singletons");

  core::DrivebrainControllerInterface::create(); 

  spdlog::info("Constructed drivebrain controller interface");

  _acu_core_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::ACUCoreData>>(_io_context, 7777);
  _acu_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::ACUAllData>>(_io_context, 7766);
  _vcr_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::VCRData_s>>(_io_context, 9999);
  _vcf_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::VCFData_s>>(_io_context, 4444);

  spdlog::info("Initialized ethernet drivers");

#if HOOTL_ENABLED
  comms::SimComms::create(); 
  comms::SimComms::instance().start();
  spdlog::info("Initialized simcomms instance");
#endif

  bool vn_init_not_successful;
  _vn_driver = std::make_unique<comms::VNDriver>(_io_context, vn_init_not_successful);
  if (vn_init_not_successful) {
    spdlog::error("Failed to initialize vectornav driver");
  }

  // CAN device names are defined in the drivebrain JSON config
  _telem_can = std::make_unique<comms::CANComms>(core::FoxgloveServer::instance().get_param<std::string>("telem_can_device").value(), _dbc_path);
  _aux_can = std::make_unique<comms::CANComms>(core::FoxgloveServer::instance().get_param<std::string>("aux_can_device").value(), _dbc_path);
  spdlog::info("Initialized CAN drivers");

  // Initialize controllers
  const size_t num_controllers = 1 + matlab_model_gen::num_controllers;
    _mode1 = std::make_shared<control::LoadCellTorqueController>(); 
  if (!_mode1->init()) {
    spdlog::error("Failed to initialize mode 1");
  }

  // Estimator Manager
  _estim_manager = std::make_shared<estimation::EstimatorManager>();
  _estim_manager->handle_inits();
  spdlog::info("Constructed estimator manager");

  std::array<std::shared_ptr<control::Controller<core::ControllerOutput, core::VehicleState>>, num_controllers> controllers{_mode1};
  auto _gend_controllers =  matlab_model_gen::create_controllers(_estim_manager);
  if (_gend_controllers.size() + 1 != controllers.size()) {
    throw std::runtime_error("Failed to initialize matlab generated controllers! Wrong vector size!");
  }
  std::copy(_gend_controllers.begin(), _gend_controllers.end(), controllers.begin() + 1);
  
  // Create controller manager instance
  ControllerManager<control::Controller<ControllerOutput, VehicleState>, num_controllers>::create(controllers);
  if(!ControllerManager<control::Controller<ControllerOutput, VehicleState>, num_controllers>::instance().init()) {
    throw std::runtime_error("Failed to initialize controller manager");
  }

  spdlog::info("Constructed controller manager");

  if (_driving_mode == DrivingMode::DRIVERLESS || _driving_mode == DrivingMode::TELEOP) {
    _autonomy.start();
  }

  running = true;

  _io_context_thread = std::thread([this]() {
    try {
      _io_context.run();
    } catch (const std::exception& e) {
      spdlog::error("io_context error: {}", e.what());
    }
   });

  _loop_thread = std::thread([this]() {
    try {
      _loop();
    } catch (const std::exception& e) {
      spdlog::error("_loop threw: {}", e.what());
    } catch (...) {
      spdlog::error("_loop threw unknown exception");
    }
    spdlog::error("_loop thread exiting, running={}", running.load());
  });

  spdlog::info("Spawned threads");
  
  // Join threads when loop thread finishes
  if(_loop_thread.joinable()) _loop_thread.join();
  _autonomy.stop();
  _io_context.stop();
  if(_io_context_thread.joinable()) _io_context_thread.join();
  spdlog::info("Joined all threads");
}

core::ControllerOutput DrivebrainApp::_teleop_command(const core::TeleopCommand& command) {
  constexpr float TELEOP_TORQUE_NM = 2.0f;
  constexpr float TELEOP_STEERING_DEG = 15.0f;

  core::TorqueControlOut torque;
  float corner_torque = command.linear_x * TELEOP_TORQUE_NM;
  torque.desired_torques_nm = {corner_torque, corner_torque, corner_torque, corner_torque};

  core::ControllerOutput out;
  out.out = torque;
  out.desired_steering_deg = command.angular_z * TELEOP_STEERING_DEG;
  return out;
}

void DrivebrainApp::_actuate(const core::ControllerOutput& out) {
  static auto desired_rpm_msg = std::make_shared<hytech::drivebrain_speed_set_input>();
  static auto torque_limit_msg = std::make_shared<hytech::drivebrain_torque_lim_input>();
  static auto desired_torque_msg = std::make_shared<hytech::drivebrain_desired_torque_input>();
  static auto steering_msg = std::make_shared<hytech::drivebrain_steering_input>();

  if (const core::SpeedControlOut* speedControl = std::get_if<core::SpeedControlOut>(&out.out)) {
    desired_rpm_msg->set_drivebrain_set_rpm_fl(speedControl->desired_rpms.FL);
    desired_rpm_msg->set_drivebrain_set_rpm_fr(speedControl->desired_rpms.FR);
    desired_rpm_msg->set_drivebrain_set_rpm_rl(speedControl->desired_rpms.RL);
    desired_rpm_msg->set_drivebrain_set_rpm_rr(speedControl->desired_rpms.RR);

    torque_limit_msg->set_drivebrain_torque_fl(speedControl->torque_lim_nm.FL);
    torque_limit_msg->set_drivebrain_torque_fr(speedControl->torque_lim_nm.FR);
    torque_limit_msg->set_drivebrain_torque_rl(speedControl->torque_lim_nm.RL);
    torque_limit_msg->set_drivebrain_torque_rr(speedControl->torque_lim_nm.RR);

    _telem_can->send_message(desired_rpm_msg);
    _telem_can->send_message(torque_limit_msg);
    _aux_can->send_message(desired_rpm_msg);
    _aux_can->send_message(torque_limit_msg);

    core::log(desired_rpm_msg);
    core::log(torque_limit_msg);

  } else if (const core::TorqueControlOut* torqueControl = std::get_if<core::TorqueControlOut>(&out.out)) {
    desired_torque_msg->set_drivebrain_torque_fl(torqueControl->desired_torques_nm.FL);
    desired_torque_msg->set_drivebrain_torque_fr(torqueControl->desired_torques_nm.FR);
    desired_torque_msg->set_drivebrain_torque_rl(torqueControl->desired_torques_nm.RL);
    desired_torque_msg->set_drivebrain_torque_rr(torqueControl->desired_torques_nm.RR);

    _telem_can->send_message(desired_torque_msg);
    _aux_can->send_message(desired_torque_msg);

    core::log(desired_torque_msg);
  }

  if (out.desired_steering_deg) {
    steering_msg->set_drivebrain_steering(*out.desired_steering_deg);

    _telem_can->send_message(steering_msg);
    _aux_can->send_message(steering_msg);

    core::log(steering_msg);
  }
}

void DrivebrainApp::_loop() {
  auto loop_time = 0.004;
  std::chrono::microseconds loop_time_ms((int) (loop_time * 1000000.0f));
  auto next_tick = std::chrono::steady_clock::now();

  while(running) {

    // spdlog::info("tick: start");
    
    next_tick += loop_time_ms;

    // spdlog::info("tick: get_state");

    auto state_and_validity = core::StateTracker::instance().vehicle_state();

    // spdlog::info("tick: evaluate_estimators");

    _estim_manager->evaluate_all_estimators(state_and_validity.first);

    core::ControllerOutput out_struct;
    bool can_command;
    if (_driving_mode == DrivingMode::DRIVERLESS) {
      out_struct = _autonomy.command(state_and_validity.first);
      can_command = _autonomy.is_valid();
    } else if (_driving_mode == DrivingMode::TELEOP) {
      auto teleop_and_validity = core::StateTracker::instance().teleop_command();
      out_struct = _teleop_command(teleop_and_validity.first);
      can_command = teleop_and_validity.second;
    } else {
      auto& controller_manager = ControllerManager<control::Controller<ControllerOutput, VehicleState>, 1 + matlab_model_gen::num_controllers>::instance();
      out_struct = controller_manager.step_active_controller(state_and_validity.first);
      can_command = state_and_validity.second;
    }

    core::StateTracker::instance().set_previous_control_output(out_struct);

    if (can_command) {
      _actuate(out_struct);
    }

    std::tuple<std::string, bool> mcap_status = core::MCAPLogger::instance().status();
    std::string logile_name = std::get<0>(mcap_status);

    std::shared_ptr<hytech_msgs::McapInfo> mcap_info = std::make_shared<hytech_msgs::McapInfo>();
    mcap_info->set_current_mcap(logile_name);

    core::log(mcap_info);

    auto now = std::chrono::steady_clock::now();
    if(now > next_tick) {
      spdlog::warn("Loop overrun by {}", now-next_tick);
      next_tick = now;
    }

    std::this_thread::sleep_until(next_tick);

    // spdlog::info("tick: stop");
  }

}
