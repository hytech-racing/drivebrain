#include "TeleopController.hpp"

#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>

#include <spdlog/spdlog.h>

control::TeleopController::~TeleopController()
{
    _running = false;
    if (_joystick_thread.joinable())
    {
        _joystick_thread.join();
    }
    if (_fd >= 0)
    {
        close(_fd);
    }
}

void control::TeleopController::_handle_param_updates(const std::unordered_map<std::string, DBParam> &new_param_map) {

    spdlog::info("Entering teleop controller parameter updates.");

    if (auto v = process_param_update<float>(new_param_map, "teleopcontroller/max_torque")) {
        std::unique_lock lk(_config_mutex);
        _config.max_torque = *v;
    }

    if (auto v = process_param_update<float>(new_param_map, "teleopcontroller/max_regen_torque")) {
        std::unique_lock lk(_config_mutex);
        _config.max_regen_torque = *v;
    }

    if (auto v = process_param_update<float>(new_param_map, "teleopcontroller/max_steering_angle_deg")) {
        std::unique_lock lk(_config_mutex);
        _config.max_steering_angle_deg = *v;
    }

    if (auto v = process_param_update<float>(new_param_map, "teleopcontroller/dt_rate_hz")) {
        std::unique_lock lk(_config_mutex);
        _config.dt_rate_hz = *v;
    }

    spdlog::info("Exiting teleop controller parameter updates.");
}

bool control::TeleopController::init()
{
    auto& foxglove = FoxgloveServer::instance();

    auto opt_max_torque             = foxglove.get_param<float>("teleopcontroller/max_torque");
    auto opt_max_regen_torque       = foxglove.get_param<float>("teleopcontroller/max_regen_torque");
    auto opt_max_steering_angle_deg = foxglove.get_param<float>("teleopcontroller/max_steering_angle_deg");
    auto opt_dt_rate_hz             = foxglove.get_param<float>("teleopcontroller/dt_rate_hz");

    bool all_loaded = true;
    if (!opt_max_torque)             { spdlog::error("Missing param: teleopcontroller/max_torque");             all_loaded = false; }
    if (!opt_max_regen_torque)       { spdlog::error("Missing param: teleopcontroller/max_regen_torque");       all_loaded = false; }
    if (!opt_max_steering_angle_deg) { spdlog::error("Missing param: teleopcontroller/max_steering_angle_deg"); all_loaded = false; }
    if (!opt_dt_rate_hz)             { spdlog::error("Missing param: teleopcontroller/dt_rate_hz");             all_loaded = false; }

    if (!all_loaded) {
        spdlog::error("Couldn't load all params for teleop controller.");
        return false;
    }

    _config.max_torque             = opt_max_torque.value();
    _config.max_regen_torque       = opt_max_regen_torque.value();
    _config.max_steering_angle_deg = opt_max_steering_angle_deg.value();
    _config.dt_rate_hz             = opt_dt_rate_hz.value();

    _config.joystick_device_path = foxglove.get_param<std::string>("teleopcontroller/joystick_device_path").value_or("/dev/input/js0");
    _config.accel_axis           = foxglove.get_param<int>("teleopcontroller/accel_axis").value_or(5);
    _config.brake_axis           = foxglove.get_param<int>("teleopcontroller/brake_axis").value_or(2);
    _config.steering_axis        = foxglove.get_param<int>("teleopcontroller/steering_axis").value_or(0);
    _config.invert_accel_axis    = foxglove.get_param<bool>("teleopcontroller/invert_accel_axis").value_or(false);
    _config.invert_brake_axis    = foxglove.get_param<bool>("teleopcontroller/invert_brake_axis").value_or(false);
    _config.invert_steering_axis = foxglove.get_param<bool>("teleopcontroller/invert_steering_axis").value_or(false);
    _config.axis_deadzone        = foxglove.get_param<float>("teleopcontroller/axis_deadzone").value_or(0.05f);
    _config.stale_timeout_ms     = foxglove.get_param<int>("teleopcontroller/stale_timeout_ms").value_or(250);

    foxglove.register_param_callback(std::bind(&control::TeleopController::_handle_param_updates, this, std::placeholders::_1));

    _last_event_time = std::chrono::steady_clock::time_point{};
    _running = true;
    _joystick_thread = std::thread(&TeleopController::_run_joystick, this);

    return true;
}

float control::TeleopController::_normalize_axis(int16_t raw, bool invert, float deadzone)
{
    float normalized = static_cast<float>(raw) / 32767.0f;
    normalized = std::clamp(normalized, -1.0f, 1.0f);
    if (std::fabs(normalized) < deadzone)
    {
        normalized = 0.0f;
    }
    return invert ? -normalized : normalized;
}

void control::TeleopController::_run_joystick()
{
    while (_running)
    {
        if (_fd < 0)
        {
            _fd = open(_config.joystick_device_path.c_str(), O_RDONLY | O_NONBLOCK);
            if (_fd < 0)
            {
                spdlog::warn("TeleopController: couldn't open {}: {}", _config.joystick_device_path, std::strerror(errno));
                std::this_thread::sleep_for(std::chrono::seconds(1));
                continue;
            }
            spdlog::info("TeleopController: opened {}", _config.joystick_device_path);
        }

        pollfd poll_fd{_fd, POLLIN, 0};
        int poll_result = poll(&poll_fd, 1, 500);
        if (poll_result < 0)
        {
            spdlog::warn("TeleopController: poll failed on {}: {}", _config.joystick_device_path, std::strerror(errno));
            close(_fd);
            _fd = -1;
            continue;
        }
        if (poll_result == 0)
        {
            // no event within the poll window; stale-check happens in _joystick_connected()
            continue;
        }

        js_event event{};
        ssize_t n_read = read(_fd, &event, sizeof(event));
        if (n_read != sizeof(event))
        {
            spdlog::warn("TeleopController: lost connection to {}: {}", _config.joystick_device_path, std::strerror(errno));
            close(_fd);
            _fd = -1;
            continue;
        }

        const auto event_type = event.type & ~JS_EVENT_INIT;
        if (event_type == JS_EVENT_AXIS)
        {
            if (event.number == _config.accel_axis)
            {
                _accel = _normalize_axis(event.value, _config.invert_accel_axis, _config.axis_deadzone);
            }
            else if (event.number == _config.brake_axis)
            {
                _brake = _normalize_axis(event.value, _config.invert_brake_axis, _config.axis_deadzone);
            }
            else if (event.number == _config.steering_axis)
            {
                _steering = _normalize_axis(event.value, _config.invert_steering_axis, _config.axis_deadzone);
            }
        }
        _last_event_time = std::chrono::steady_clock::now();
    }
}

bool control::TeleopController::_joystick_connected() const
{
    if (_fd < 0)
    {
        return false;
    }
    auto age = std::chrono::steady_clock::now() - _last_event_time.load();
    return age <= std::chrono::milliseconds(_config.stale_timeout_ms);
}

ControllerOutput control::TeleopController::step_controller(const VehicleState &in)
{
    config cur_config;
    {
        std::unique_lock lk(_config_mutex);
        cur_config = _config;
    }

    TorqueControlOut torque_out{};
    ControllerOutput cmd_out{};

    const bool safe_to_drive = _joystick_connected();

    float steering_cmd_deg = 0.0f;

    if (safe_to_drive) {
        const float accel = _accel;
        const float brake = _brake;
        const float steering = std::clamp(_steering.load(), -1.0f, 1.0f);

        const torque_nm requested_torque = (accel * cur_config.max_torque) - (brake * cur_config.max_regen_torque);

        torque_out.desired_torques_nm.FL = requested_torque;
        torque_out.desired_torques_nm.FR = requested_torque;
        torque_out.desired_torques_nm.RL = requested_torque;
        torque_out.desired_torques_nm.RR = requested_torque;

        steering_cmd_deg = steering * cur_config.max_steering_angle_deg;
    } else {
        torque_out.desired_torques_nm = {0.0f, 0.0f, 0.0f, 0.0f};
    }

    cmd_out.out = torque_out;
    cmd_out.steering_angle_deg_cmd = steering_cmd_deg;

    return cmd_out;
}
