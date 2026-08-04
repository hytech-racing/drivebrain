#include "app.hpp"

#include <fmt/chrono.h>
#include <spdlog/spdlog.h>

#include <StateTracker.hpp>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <mcap/writer.hpp>
#include <memory>
#include <stdexcept>

#include "ConfigParamLoader.hpp"
#include "ControllerManager.hpp"
#include "DrivebrainControllerInterface.hpp"
#include "ETHRecvComms.hpp"
#include "EstimatorMessageAdapters.hpp"
#include "FoxgloveServer.hpp"
#include "MCAPLogger.hpp"
#include "MatlabModelAddHelper.hpp"
#include "PathPlanner.hpp"
#include "PointCloudMessageAdapters.hpp"
#include "Telemetry.hpp"
#include "dv_msgs.pb.h"
#include "hytech_msgs.pb.h"

std::atomic<bool> running{true};

void sig_handler(int signal)
{
    if (signal == SIGINT)
    {
        spdlog::warn("Interrupted, stopping Drivebrain app");
        running = false;
    }
}

DrivebrainApp::DrivebrainApp(const std::string& json_param_path,
                             const std::string& dbc_path)
    : _json_params_path(json_param_path), _dbc_path(dbc_path)
{
}

DrivebrainApp::~DrivebrainApp()
{
    running = false;

    core::MCAPLogger::instance().destroy();
    core::FoxgloveServer::instance().destroy();
    spdlog::info("Logging singletons safely destroyed");
}

void DrivebrainApp::run()
{
    std::signal(SIGINT, sig_handler);

    core::MCAPLogger::create("recordings/", mcap::McapWriterOptions(""),
                             _json_params_path);
    core::FoxgloveServer::create(_json_params_path);
    core::StateTracker::create();

    core::MCAPLogger::instance().open_new_mcap();
    core::MCAPLogger::instance().init_logging();

    spdlog::info("Constructed logging singletons");

    core::DrivebrainControllerInterface::create();

    spdlog::info("Constructed drivebrain controller interface");

    _acu_core_eth_driver =
        std::make_unique<comms::ETHRecvComms<hytech_msgs::ACUCoreData>>(
            _io_context, 7777);
    _acu_eth_driver =
        std::make_unique<comms::ETHRecvComms<hytech_msgs::ACUAllData>>(
            _io_context, 7766);
    _vcr_eth_driver =
        std::make_unique<comms::ETHRecvComms<hytech_msgs::VCRData_s>>(
            _io_context, 9999);
    _vcf_eth_driver =
        std::make_unique<comms::ETHRecvComms<hytech_msgs::VCFData_s>>(
            _io_context, 4444);

    spdlog::info("Initialized ethernet drivers");

    _latest_estimate = std::make_shared<estimation::LatestEstimate>();

    // TransformBuffer will be used as the source of truth for all transforms,
    // so set the static transforms immediately
    _transform_buffer =
        std::make_shared<transforms::TransformBuffer>(5'000'000'000ULL);
    const app_config::StaticTransformParams static_transform_params =
        app_config::load_static_transform_params();
    _transform_buffer->set_T_base_imu3d(static_transform_params.T_base_imu);
    _transform_buffer->set_T_base_gss3d(static_transform_params.T_base_gss);
    _transform_buffer->set_T_base_lidar3d(static_transform_params.T_base_lidar);

    const app_config::DriverlessEstimatorRunnerParams estimator_params =
        app_config::load_driverless_estimator_runner_params();

    _driverless_estimator_runner =
        std::make_unique<runtime::DriverlessEstimatorRunner>(
            _latest_estimate, _transform_buffer, estimator_params.ekf_params,
            estimator_params.gss_sensor_config);
    _driverless_estimator_runner->start();

    spdlog::info("Started driverless estimator runner");

    _latest_map_state = std::make_shared<slam::LatestMapState>();
    _latest_planner_map = std::make_shared<slam::LatestPlannerMap>();

    const slam::backend::IncrementalGraphSlamParams graph_slam_params =
        app_config::load_incremental_graph_slam_params();
    _slam_backend_runner = std::make_unique<runtime::SlamBackendRunner>(
        _latest_map_state, graph_slam_params, true);
    _slam_backend_runner->start();
    spdlog::info("Started graphSLAM backend runner");

    perception::LidarProcessorParams lidar_processor_params =
        app_config::load_lidar_processor_params();
    const slam::frontend::SlamFrontendParams slam_frontend_params =
        app_config::load_slam_frontend_params();
    _perception_frontend_runner =
        std::make_unique<runtime::PerceptionFrontendRunner>(
            _transform_buffer, _latest_map_state,
            [this](slam::LandmarkFrame frame)
            { return _slam_backend_runner->enqueue(std::move(frame)); },
            lidar_processor_params, slam_frontend_params, true,
            _latest_planner_map);
    _perception_frontend_runner->start();
    spdlog::info("Started perception frontend runner");

#if HOOTL_ENABLED
    comms::SimComms::create(
        [this](std::shared_ptr<google::protobuf::Message> msg)
        { _route_received_message(msg); });
    comms::SimComms::instance().start();
#endif

    bool vn_init_not_successful;
    _vn_driver =
        std::make_unique<comms::VNDriver>(_io_context, vn_init_not_successful);
    if (vn_init_not_successful)
    {
        spdlog::error("Failed to initialize vectornav driver");
    }

    // CAN device names are defined in the drivebrain JSON config
    _telem_can = std::make_unique<comms::CANComms>(
        core::FoxgloveServer::instance()
            .get_param<std::string>("telem_can_device")
            .value(),
        _dbc_path);
    _aux_can = std::make_unique<comms::CANComms>(
        core::FoxgloveServer::instance()
            .get_param<std::string>("aux_can_device")
            .value(),
        _dbc_path);
    spdlog::info("Initialized CAN drivers");

    // Initialize controllers
    const size_t num_controllers = 1 + matlab_model_gen::num_controllers;
    _mode1 = std::make_shared<control::LoadCellTorqueController>();
    if (!_mode1->init())
    {
        spdlog::error("Failed to initialize mode 1");
    }

    // Estimator Manager
    _estim_manager = std::make_shared<estimation::EstimatorManager>();
    _estim_manager->handle_inits();
    spdlog::info("Constructed estimator manager");

    std::array<std::shared_ptr<control::Controller<core::ControllerOutput,
                                                   core::VehicleState>>,
               num_controllers>
        controllers{_mode1};
    auto _gend_controllers =
        matlab_model_gen::create_controllers(_estim_manager);
    if (_gend_controllers.size() + 1 != controllers.size())
    {
        throw std::runtime_error(
            "Failed to initialize matlab generated controllers! Wrong vector "
            "size!");
    }
    std::copy(_gend_controllers.begin(), _gend_controllers.end(),
              controllers.begin() + 1);

    // Create controller manager instance
    ControllerManager<control::Controller<ControllerOutput, VehicleState>,
                      num_controllers>::create(controllers);
    if (!ControllerManager<control::Controller<ControllerOutput, VehicleState>,
                           num_controllers>::instance()
             .init())
    {
        throw std::runtime_error("Failed to initialize controller manager");
    }

    spdlog::info("Constructed controller manager");

    if (_driving_mode == DrivingMode::DRIVERLESS ||
        _driving_mode == DrivingMode::TELEOP)
    {
        _autonomy.start();
    }

    running = true;

    _io_context_thread = std::thread(
        [this]()
        {
            try
            {
                _io_context.run();
            }
            catch (const std::exception& e)
            {
                spdlog::error("io_context error: {}", e.what());
            }
        });

    _loop_thread = std::thread(
        [this]()
        {
            try
            {
                _loop();
            }
            catch (const std::exception& e)
            {
                spdlog::error("_loop threw: {}", e.what());
            }
            catch (...)
            {
                spdlog::error("_loop threw unknown exception");
            }
            spdlog::error("_loop thread exiting, running={}", running.load());
        });

    spdlog::info("Spawned threads");

    // Join threads when loop thread finishes
    if (_loop_thread.joinable()) _loop_thread.join();

#if HOOTL_ENABLED
    spdlog::info("Stopping SimComms");
    comms::SimComms::instance().close();
    comms::SimComms::destroy();
#endif

    if (_perception_frontend_runner)
    {
        spdlog::info("Stopping perception frontend runner");
        _perception_frontend_runner->stop();
    }

    if (_slam_backend_runner)
    {
        spdlog::info("Stopping graphSLAM backend runner");
        _slam_backend_runner->stop();
    }

    spdlog::info("Stopping autonomy stack");
    _autonomy.stop();

    if (_driverless_estimator_runner)
    {
        spdlog::info("Stopping driverless estimator");
        _driverless_estimator_runner->stop();
    }

    _io_context.stop();
    if (_io_context_thread.joinable()) _io_context_thread.join();
    spdlog::info("Joined all threads");
}

core::ControllerOutput DrivebrainApp::_teleop_command(
    const core::TeleopCommand& command)
{
    constexpr float TELEOP_TORQUE_NM = 2.0f;
    constexpr float TELEOP_STEERING_DEG = 15.0f;

    core::TorqueControlOut torque;
    float corner_torque = command.linear_x * TELEOP_TORQUE_NM;
    torque.desired_torques_nm = {corner_torque, corner_torque, corner_torque,
                                 corner_torque};

    core::ControllerOutput out;
    out.out = torque;
    out.desired_steering_deg = command.angular_z * TELEOP_STEERING_DEG;
    return out;
}

void DrivebrainApp::_actuate(const core::ControllerOutput& out)
{
    static auto desired_rpm_msg =
        std::make_shared<hytech::drivebrain_speed_set_input>();
    static auto torque_limit_msg =
        std::make_shared<hytech::drivebrain_torque_lim_input>();
    static auto desired_torque_msg =
        std::make_shared<hytech::drivebrain_desired_torque_input>();
    static auto steering_msg =
        std::make_shared<hytech::drivebrain_steering_input>();

    if (const core::SpeedControlOut* speedControl =
            std::get_if<core::SpeedControlOut>(&out.out))
    {
        desired_rpm_msg->set_drivebrain_set_rpm_fl(
            speedControl->desired_rpms.FL);
        desired_rpm_msg->set_drivebrain_set_rpm_fr(
            speedControl->desired_rpms.FR);
        desired_rpm_msg->set_drivebrain_set_rpm_rl(
            speedControl->desired_rpms.RL);
        desired_rpm_msg->set_drivebrain_set_rpm_rr(
            speedControl->desired_rpms.RR);

        torque_limit_msg->set_drivebrain_torque_fl(
            speedControl->torque_lim_nm.FL);
        torque_limit_msg->set_drivebrain_torque_fr(
            speedControl->torque_lim_nm.FR);
        torque_limit_msg->set_drivebrain_torque_rl(
            speedControl->torque_lim_nm.RL);
        torque_limit_msg->set_drivebrain_torque_rr(
            speedControl->torque_lim_nm.RR);

        _telem_can->send_message(desired_rpm_msg);
        _telem_can->send_message(torque_limit_msg);
        _aux_can->send_message(desired_rpm_msg);
        _aux_can->send_message(torque_limit_msg);

        core::log(desired_rpm_msg);
        core::log(torque_limit_msg);
    }
    else if (const core::TorqueControlOut* torqueControl =
                 std::get_if<core::TorqueControlOut>(&out.out))
    {
        desired_torque_msg->set_drivebrain_torque_fl(
            torqueControl->desired_torques_nm.FL);
        desired_torque_msg->set_drivebrain_torque_fr(
            torqueControl->desired_torques_nm.FR);
        desired_torque_msg->set_drivebrain_torque_rl(
            torqueControl->desired_torques_nm.RL);
        desired_torque_msg->set_drivebrain_torque_rr(
            torqueControl->desired_torques_nm.RR);

        _telem_can->send_message(desired_torque_msg);
        _aux_can->send_message(desired_torque_msg);

        core::log(desired_torque_msg);
    }

    if (out.desired_steering_deg)
    {
        steering_msg->set_drivebrain_steering(*out.desired_steering_deg);

        _telem_can->send_message(steering_msg);
        _aux_can->send_message(steering_msg);

        core::log(steering_msg);
    }
}

void DrivebrainApp::_loop()
{
    auto loop_time = 0.004;
    std::chrono::microseconds loop_time_ms((int)(loop_time * 1000000.0f));
    auto next_tick = std::chrono::steady_clock::now();

    auto previous_print_time = next_tick;

    using namespace std::chrono_literals;

    while (running)
    {
        // spdlog::info("tick: start");

        next_tick += loop_time_ms;

        // {
        //     auto now_time = std::chrono::steady_clock::now();
        //     auto elapsed = now_time - previous_print_time;
        //
        //     auto ms =
        //     std::chrono::duration_cast<std::chrono::milliseconds>(elapsed);
        //
        //     if (ms > 1000ms) {
        //         const auto stats = _driverless_estimator->stats();
        //
        //         spdlog::info(
        //         "estimator ingress: imu={}/{}, gss={}/{}, "
        //         "depth={}, max_depth={}, drops={}, out_of_order={}",
        //         stats.imu_processed,
        //         stats.imu_enqueued,
        //         stats.gss_processed,
        //         stats.gss_enqueued,
        //         stats.current_queue_depth,
        //         stats.maximum_queue_depth,
        //         stats.queue_drops,
        //         stats.out_of_order_measurements);
        //
        //         previous_print_time = now_time;
        //     }
        // }

        // spdlog::info("tick: get_state");

        auto state_and_validity =
            core::StateTracker::instance().vehicle_state();

        // spdlog::info("tick: evaluate_estimators");

        _estim_manager->evaluate_all_estimators(state_and_validity.first);

        core::ControllerOutput out_struct;
        bool can_command;
        if (_driving_mode == DrivingMode::DRIVERLESS)
        {
            out_struct = _autonomy.command(state_and_validity.first);
            can_command = _autonomy.is_valid();
        }
        else if (_driving_mode == DrivingMode::TELEOP)
        {
            auto teleop_and_validity =
                core::StateTracker::instance().teleop_command();
            out_struct = _teleop_command(teleop_and_validity.first);
            can_command = teleop_and_validity.second;
        }
        else
        {
            auto& controller_manager = ControllerManager<
                control::Controller<ControllerOutput, VehicleState>,
                1 + matlab_model_gen::num_controllers>::instance();
            out_struct = controller_manager.step_active_controller(
                state_and_validity.first);
            can_command = state_and_validity.second;
        }

        core::StateTracker::instance().set_previous_control_output(out_struct);

        if (can_command)
        {
            _actuate(out_struct);
        }

        std::tuple<std::string, bool> mcap_status =
            core::MCAPLogger::instance().status();
        std::string logile_name = std::get<0>(mcap_status);

        std::shared_ptr<hytech_msgs::McapInfo> mcap_info =
            std::make_shared<hytech_msgs::McapInfo>();
        mcap_info->set_current_mcap(logile_name);

        core::log(mcap_info);

        auto now = std::chrono::steady_clock::now();
        if (now > next_tick)
        {
            spdlog::warn("Loop overrun by {}", now - next_tick);
            next_tick = now;
        }

        std::this_thread::sleep_until(next_tick);

        // spdlog::info("tick: stop");
    }
}

void DrivebrainApp::_route_received_message(
    std::shared_ptr<google::protobuf::Message> message)
{
    const auto* descriptor = message->GetDescriptor();

    if (descriptor == hytech_msgs::VnImuData::descriptor())
    {
        const auto typed =
            std::static_pointer_cast<hytech_msgs::VnImuData>(message);

        const auto measurement = adapters::to_imu_measurement(*typed);

        if (!measurement)
        {
            spdlog::warn("Rejected invalid VnImuData");
        }

        if (measurement)
        {
            if (!_driverless_estimator_runner->enqueue(*measurement))
            {
                spdlog::warn("Failed to enqueue IMU measurement at {} ns",
                             measurement->timestamp_ns);
            }
        }

        return;
    }

    if (descriptor == hytech_msgs::GssData::descriptor())
    {
        const auto typed =
            std::static_pointer_cast<hytech_msgs::GssData>(message);

        const auto measurement = adapters::to_gss_measurement(*typed);

        if (!measurement)
        {
            spdlog::warn("Rejected invalid GssData");
        }

        if (measurement)
        {
            if (!_driverless_estimator_runner->enqueue(*measurement))
            {
                spdlog::warn("Failed to enqueue GSS measurement at {} ns",
                             measurement->timestamp_ns);
            }
        }

        return;
    }

    if (descriptor == foxglove::PointCloud::descriptor())
    {
        const auto typed =
            std::static_pointer_cast<foxglove::PointCloud>(message);

        auto point_cloud = adapters::to_core_point_cloud(*typed);

        if (point_cloud)
        {
            const auto timestamp_ns = point_cloud->timestamp_ns;
            if (!_perception_frontend_runner->enqueue(std::move(*point_cloud)))
            {
                spdlog::warn("Failed to enqueue PointCloud at {} ns",
                             timestamp_ns);
            }
        }
    }

    core::StateTracker::instance().handle_receive_protobuf_message(
        std::move(message));
}
