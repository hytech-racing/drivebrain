#include "drivebrain_app.hpp"
#include "ETHRecvComms.hpp"
#include "FoxgloveServer.hpp"
#include "MCAPLogger.hpp"
#include "OusterComms.hpp"
#include "hytech_msgs.pb.h"
#include "dv_msgs.pb.h"
#include "Telemetry.hpp"
#include "ControllerManager.hpp"
#include "DrivebrainControllerInterface.hpp"
#include "PathPlanner.hpp"
#include <StateTracker.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <mcap/writer.hpp>
#include <memory>
#include <fmt/chrono.h>
#include <spdlog/spdlog.h>
#include <filesystem>
#include <stdexcept>


std::atomic<bool> running{true};

void sig_handler(int signal) {
    if(signal == SIGINT || signal == SIGTERM) {
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
  std::signal(SIGTERM, sig_handler);

  core::MCAPLogger::create("recordings/", mcap::McapWriterOptions(""), _json_params_path);
  core::FoxgloveServer::create(_json_params_path);
  core::StateTracker::create();

  core::MCAPLogger::instance().open_new_mcap();
  core::MCAPLogger::instance().init_logging();

  spdlog::info("Constructed logging singletons");

  _acu_core_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::ACUCoreData>>(_io_context, 7777);
  _acu_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::ACUAllData>>(_io_context, 7766);
  _vcr_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::VCRData_s>>(_io_context, 9999);
  _vcf_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::VCFData_s>>(_io_context, 4444);
  _remote_eth_driver = std::make_unique<comms::ETHRecvComms<hytech_msgs::RemoteDriveCommand>>(_io_context, 5556,
    [](std::shared_ptr<hytech_msgs::RemoteDriveCommand> msg) {
      core::log(msg);
      // copied, the driver reuses msg for the next receive
      core::StateTracker::instance().set_remote_command(*msg);
    });

  spdlog::info("Initialized ethernet drivers");

#if !HOOTL_ENABLED
  if (core::FoxgloveServer::instance().get_param<bool>("oms/enabled").value_or(false)) {
    auto oms_host = core::FoxgloveServer::instance().get_param<std::string>("oms/host");
    auto oms_port = core::FoxgloveServer::instance().get_param<int>("oms/port");
    if (oms_host && oms_port) {
      _oms_comms = std::make_unique<comms::OMSComms>(_io_context, oms_host.value(), static_cast<uint16_t>(oms_port.value()));
    } else {
      spdlog::error("Couldn't load oms/host and oms/port");
    }
  }
#endif

#if HOOTL_ENABLED
  comms::SimComms::create();
  comms::SimComms::instance().start();
#endif

  bool vn_init_not_successful;
  _vn_driver = std::make_unique<comms::VNDriver>(_io_context, vn_init_not_successful);
  if (vn_init_not_successful) {
    spdlog::error("Failed to initialize vectornav driver");
  }

#if JETSON_ENABLED
  _ouster_driver = std::make_unique<comms::OusterComms>();
  spdlog::info("ouster driver init");
#endif

#if !HOOTL_ENABLED
  _kraken_comms = std::make_unique<comms::KrakenComms>();
  if (!_kraken_comms->init()) {
    spdlog::error("Failed to initialize KrakenComms");
    _kraken_comms.reset();
  }
#endif


  // CAN device names are defined in the drivebrain JSON config
  _telem_can = std::make_unique<comms::CANComms>(core::FoxgloveServer::instance().get_param<std::string>("telem_can_device").value(), _dbc_path);
  _aux_can = std::make_unique<comms::CANComms>(core::FoxgloveServer::instance().get_param<std::string>("aux_can_device").value(), _dbc_path);
  spdlog::info("Initialized CAN drivers");

  // TODO FIX HOW THIS IS AIDS IF A CAM IS NOT PLUGGED IN
#if JETSON_ENABLED
  // start every GigE camera aravis can find
  constexpr unsigned int expected_cameras = 2;
  unsigned int num_cameras = 0;
  for (int attempt = 0; attempt < 15; ++attempt) {
    arv_update_device_list();
    num_cameras = arv_get_n_devices();
    if (num_cameras >= expected_cameras) break;
    std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  spdlog::info("Found {} cameras", num_cameras);
  for (unsigned int i = 0; i < num_cameras; i++) {
    auto camera = std::make_unique<comms::BlackflyComms>();
    camera->start(arv_get_device_id(i), std::string("blackfly_") + arv_get_device_serial_nbr(i), "BayerRG8", 15.0);
    _camera_drivers.push_back(std::move(camera));
  }
#endif

  // Initialize controllers
  const size_t num_controllers = 2;
  _mode1 = std::make_shared<control::LoadCellTorqueController>();
  if (!_mode1->init()) {
    spdlog::error("Failed to initialize mode 1");
  }

  _mode2 = std::make_shared<control::TeleopController>();
  if (!_mode2->init()) {
    spdlog::error("Failed to initialize mode 2");
  }

  std::array<std::shared_ptr<control::Controller<core::ControllerOutput, core::VehicleState>>, num_controllers> controllers{_mode1, _mode2};

  // Create controller manager instance
  ControllerManager<control::Controller<core::ControllerOutput, core::VehicleState>, num_controllers>::create(controllers);
  if(!ControllerManager<control::Controller<core::ControllerOutput, core::VehicleState>, num_controllers>::instance().init()) {
    throw std::runtime_error("Failed to initialize controller manager");
  }

  spdlog::info("Constructed controller manager");

  core::DrivebrainControllerInterface::create();

  spdlog::info("Constructed drivebrain controller interface");
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

core::ControllerOutput DrivebrainApp::_remote_command(const core::RemoteCommand& command, bool is_fresh, const core::VehicleState& vehicle_state) {
  constexpr float REMOTE_MAX_TORQUE_NM = 2.0f;

  core::SpeedControlOut speed{};
  speed.desired_rpms = {0.0f, 0.0f, 0.0f, 0.0f};
  speed.torque_lim_nm = {REMOTE_MAX_TORQUE_NM, REMOTE_MAX_TORQUE_NM, REMOTE_MAX_TORQUE_NM, REMOTE_MAX_TORQUE_NM};
  speed.mcu_recv_millis = vehicle_state.prev_MCU_recv_millis;

  core::ControllerOutput out;
  if (is_fresh) {
    const auto& rpms = command.msg.desired_rpms();
    const auto& torque_lim = command.msg.torque_limit_nm();
    speed.desired_rpms = {rpms.fl(), rpms.fr(), rpms.rl(), rpms.rr()};
    speed.torque_lim_nm = {std::clamp(torque_lim.fl(), 0.0f, REMOTE_MAX_TORQUE_NM),
                           std::clamp(torque_lim.fr(), 0.0f, REMOTE_MAX_TORQUE_NM),
                           std::clamp(torque_lim.rl(), 0.0f, REMOTE_MAX_TORQUE_NM),
                           std::clamp(torque_lim.rr(), 0.0f, REMOTE_MAX_TORQUE_NM)};

    if (command.msg.has_desired_steering_deg()) {
      out.desired_steering_deg = command.msg.desired_steering_deg();
    }
  }

  out.out = speed;
  return out;
}

void DrivebrainApp::_actuate(const core::ControllerOutput& out) {
  static auto desired_rpm_msg = std::make_shared<hytech::drivebrain_speed_set_input>();
  static auto torque_limit_msg = std::make_shared<hytech::drivebrain_torque_lim_input>();
  static auto desired_torque_msg = std::make_shared<hytech::drivebrain_desired_torque_input>();
  static auto steering_msg = std::make_shared<hytech::drivebrain_steering_input>();

  // every torque and rpm sent to the inverters is capped here, in all driving modes
  constexpr float MAX_TORQUE_NM = 2.0f;
  constexpr float MAX_RPM = 5000.0f;

  if (const core::SpeedControlOut* speedControl = std::get_if<core::SpeedControlOut>(&out.out)) {
    desired_rpm_msg->set_drivebrain_set_rpm_fl(std::clamp(speedControl->desired_rpms.FL, -MAX_RPM, MAX_RPM));
    desired_rpm_msg->set_drivebrain_set_rpm_fr(std::clamp(speedControl->desired_rpms.FR, -MAX_RPM, MAX_RPM));
    desired_rpm_msg->set_drivebrain_set_rpm_rl(std::clamp(speedControl->desired_rpms.RL, -MAX_RPM, MAX_RPM));
    desired_rpm_msg->set_drivebrain_set_rpm_rr(std::clamp(speedControl->desired_rpms.RR, -MAX_RPM, MAX_RPM));

    torque_limit_msg->set_drivebrain_torque_fl(std::clamp(speedControl->torque_lim_nm.FL, 0.0f, MAX_TORQUE_NM));
    torque_limit_msg->set_drivebrain_torque_fr(std::clamp(speedControl->torque_lim_nm.FR, 0.0f, MAX_TORQUE_NM));
    torque_limit_msg->set_drivebrain_torque_rl(std::clamp(speedControl->torque_lim_nm.RL, 0.0f, MAX_TORQUE_NM));
    torque_limit_msg->set_drivebrain_torque_rr(std::clamp(speedControl->torque_lim_nm.RR, 0.0f, MAX_TORQUE_NM));

    _telem_can->send_message(desired_rpm_msg);
    _telem_can->send_message(torque_limit_msg);
    _aux_can->send_message(desired_rpm_msg);
    _aux_can->send_message(torque_limit_msg);

    core::log(desired_rpm_msg);
    core::log(torque_limit_msg);

  } else if (const core::TorqueControlOut* torqueControl = std::get_if<core::TorqueControlOut>(&out.out)) {
    desired_torque_msg->set_drivebrain_torque_fl(std::clamp(torqueControl->desired_torques_nm.FL, -MAX_TORQUE_NM, MAX_TORQUE_NM));
    desired_torque_msg->set_drivebrain_torque_fr(std::clamp(torqueControl->desired_torques_nm.FR, -MAX_TORQUE_NM, MAX_TORQUE_NM));
    desired_torque_msg->set_drivebrain_torque_rl(std::clamp(torqueControl->desired_torques_nm.RL, -MAX_TORQUE_NM, MAX_TORQUE_NM));
    desired_torque_msg->set_drivebrain_torque_rr(std::clamp(torqueControl->desired_torques_nm.RR, -MAX_TORQUE_NM, MAX_TORQUE_NM));

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

    next_tick += loop_time_ms;

    // spdlog::info("tick: get_state");

    auto state_and_validity = core::StateTracker::instance().vehicle_state();

    core::ControllerOutput out_struct;
    bool command_is_valid;
    if (_driving_mode == DrivingMode::DRIVERLESS) {
      out_struct = _autonomy.command(state_and_validity.first);
      command_is_valid = _autonomy.is_valid();
    } else if (_driving_mode == DrivingMode::TELEOP) {
      auto teleop_and_validity = core::StateTracker::instance().teleop_command();
      out_struct = _teleop_command(teleop_and_validity.first);
      command_is_valid = teleop_and_validity.second;
    } else if (_driving_mode == DrivingMode::REMOTE) {
      auto remote_and_validity = core::StateTracker::instance().remote_command();
      out_struct = _remote_command(remote_and_validity.first, remote_and_validity.second, state_and_validity.first);
      // always actuate so a stale command goes out as zero instead of leaving the last one on CAN
      command_is_valid = true;
    } else {
      auto& controller_manager = ControllerManager<control::Controller<ControllerOutput, VehicleState>, 1>::instance();
      out_struct = controller_manager.step_active_controller(state_and_validity.first);
      command_is_valid = state_and_validity.second;
    }

#if !HOOTL_ENABLED
    if (_kraken_comms && _driving_mode == DrivingMode::REMOTE) {
      // remote command is already a column angle. Holds the last target when stale
      if (out_struct.desired_steering_deg) {
        _kraken_comms->set_angle(*out_struct.desired_steering_deg);
      }
    } else if (_kraken_comms && state_and_validity.second) {
      _kraken_comms->set_angle(state_and_validity.first.steering_angle_deg);
    }
#endif

    core::StateTracker::instance().set_previous_control_output(out_struct);

    if (command_is_valid) {
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
