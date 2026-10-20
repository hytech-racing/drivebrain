#include <LaunchController.hpp>
#include <Literals.hpp>

bool control::LaunchController::init() {
    auto& server = core::FoxgloveServer::instance();
    std::unordered_map<std::string, core::DBParam> params;
    for (const auto* name : {"acceleration_mps2", "accel_distance_m", "braking_distance_m",
                            "max_speed_mps", "max_torque_nm", "max_regen_torque_nm"}) {
        const std::string key = std::string("launchcontroller/") + name;
        auto value = server.get_param<float>(key);
        if (!value) return false;
        params[key] = *value;
    }
    for (const auto* name : {"heading_kp", "heading_kd", "max_steering_column_angle_deg", "steering_sign"}) {
        const std::string key = std::string("launchcontroller/") + name;
        if (auto value = server.get_param<float>(key)) params[key] = *value;
    }
    for (const auto* name : {"use_ground_speed", "heading_control_enabled"}) {
        const std::string key = std::string("launchcontroller/") + name;
        if (auto value = server.get_param<bool>(key)) params[key] = *value;
    }
    _handle_param_updates(params);
    // A true value in a saved config must never initiate a launch at startup.
    _enabled = server.get_param<bool>("launchcontroller/testing_enabled").value_or(false);
    _brake_requested = server.get_param<bool>("launchcontroller/brake_requested").value_or(false);
    _param_connection = server.register_param_callback(
        [this](const auto& updated) { _handle_param_updates(updated); });
    return _config_valid && _ground_config_valid;
}

void control::LaunchController::_handle_param_updates(
    const std::unordered_map<std::string, core::DBParam>& params) {
    std::scoped_lock lock(_mutex);
    auto candidate = _config;
    auto update = [&](const char* name, float& field) {
        auto it = params.find(std::string("launchcontroller/") + name);
        if (it != params.end()) {
            if (auto value = std::get_if<float>(&it->second)) field = *value;
        }
    };
    update("acceleration_mps2", candidate.acceleration_mps2);
    update("accel_distance_m", candidate.accel_distance_m);
    update("braking_distance_m", candidate.braking_distance_m);
    update("max_speed_mps", candidate.max_speed_mps);
    update("max_torque_nm", candidate.max_torque_nm);
    update("max_regen_torque_nm", candidate.max_regen_torque_nm);
    constexpr float max_rpm = 20000.0f;
    _config_valid = LaunchProfile::valid(candidate) &&
        candidate.max_speed_mps * constants::METERS_PER_SECOND_TO_RPM <= max_rpm;
    if (_config_valid) _config = candidate;
    else spdlog::error("LaunchController: invalid configuration; new launches disabled");
    auto read_bool = [&](const char* name, bool fallback) {
        auto it = params.find(std::string("launchcontroller/") + name);
        if (it != params.end()) {
            if (auto value = std::get_if<bool>(&it->second)) return *value;
        }
        return fallback;
    };
    const bool enabled = read_bool("testing_enabled", _enabled);
    _ground_config.use_ground_speed = read_bool("use_ground_speed", _ground_config.use_ground_speed);
    _ground_config.heading_control_enabled = read_bool("heading_control_enabled", _ground_config.heading_control_enabled);
    update("heading_kp", _ground_config.heading.kp);
    update("heading_kd", _ground_config.heading.kd);
    update("max_steering_column_angle_deg", _ground_config.heading.max_column_angle_deg);
    update("steering_sign", _ground_config.heading.steering_sign);
    _ground_config_valid = HeadingHold::valid(_ground_config.heading);
    _start_requested = _start_requested || (enabled && !_enabled);
    if (!enabled) _start_requested = false;
    _enabled = enabled;
    _brake_requested = read_bool("brake_requested", _brake_requested);
}

void control::LaunchController::reset() {
    std::scoped_lock lock(_mutex);
    _profile = LaunchProfile{};
    _last_output = {core::SpeedControlOut{}, 0.0f};
    _start_requested = false;
    _last_step = {};
}

core::ControllerOutput control::LaunchController::step_controller(const core::VehicleState& in) {
    std::scoped_lock lock(_mutex);
    const auto now = std::chrono::steady_clock::now();
    float dt = _last_step.time_since_epoch().count() == 0 ? 0.0f :
        std::chrono::duration<float>(now - _last_step).count();
    _last_step = now;
    const std::array<float, 4> speeds{
        static_cast<float>(in.current_rpms.FL * constants::RPM_TO_METERS_PER_SECOND),
        static_cast<float>(in.current_rpms.FR * constants::RPM_TO_METERS_PER_SECOND),
        static_cast<float>(in.current_rpms.RL * constants::RPM_TO_METERS_PER_SECOND),
        static_cast<float>(in.current_rpms.RR * constants::RPM_TO_METERS_PER_SECOND)};
    const std::array<float, 4> rpms{in.current_rpms.FL, in.current_rpms.FR, in.current_rpms.RL, in.current_rpms.RR};
    bool healthy = in.state_is_valid && in.is_ready_to_drive && in.wheel_speeds_valid &&
        std::all_of(rpms.begin(), rpms.end(), [](float rpm) {
            return std::isfinite(rpm) && std::abs(rpm) <= 20000.0f;
        });
    const auto previous_phase = _profile.phase();
    if (_start_requested) {
        if (previous_phase != LaunchProfile::Phase::ACCELERATING && previous_phase != LaunchProfile::Phase::BRAKING) {
            _active_ground_config = _ground_config;
            _heading_hold.reset(in.oms.heading_deg);
        }
    }
    if (_active_ground_config.use_ground_speed || _active_ground_config.heading_control_enabled) {
        healthy = healthy && in.oms.received && std::isfinite(in.oms.heading_deg) && std::isfinite(in.oms.yaw_rate_deg_s);
    }
    const std::optional<LaunchProfile::GroundMotion> ground = _active_ground_config.use_ground_speed ?
        std::optional<LaunchProfile::GroundMotion>{{in.oms.speed_mps, in.oms.distance_m}} : std::nullopt;
    if (_start_requested) {
        _profile.start(_config, healthy && _config_valid && _ground_config_valid && !_brake_requested, speeds, ground);
        _start_requested = false;
        dt = 0.0f;
    }
    const auto result = _profile.step(dt, speeds, healthy,
        _brake_requested || !_enabled || in.input.requested_brake > 0.2f, ground);
    if (_profile.phase() != previous_phase) {
        spdlog::info("LaunchController: phase={} distance={}m deceleration={}m/s^2",
            static_cast<int>(_profile.phase()), _profile.distance_m(), _profile.deceleration_mps2());
    }
    const float rpm = result.speed_mps * constants::METERS_PER_SECOND_TO_RPM;
    const float torque = result.torque_limit_nm;
    const bool running = _profile.phase() == LaunchProfile::Phase::ACCELERATING || _profile.phase() == LaunchProfile::Phase::BRAKING;
    const float steering = running && _active_ground_config.heading_control_enabled ?
        _heading_hold.step(_active_ground_config.heading, in.oms.heading_deg, in.oms.yaw_rate_deg_s) : 0.0f;
    _last_output = {core::SpeedControlOut{in.prev_MCU_recv_millis,
        {rpm, rpm, rpm, rpm}, {torque, torque, torque, torque}}, steering};
    return _last_output;
}
