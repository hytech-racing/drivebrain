#include <LaunchController.hpp>
#include <Literals.hpp>
#include <google/protobuf/struct.pb.h>

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
    for (const auto* name : {"heading_kp", "heading_kd", "max_steering_column_angle_deg", "steering_sign", "fake_heading_max_deg"}) {
        const std::string key = std::string("launchcontroller/") + name;
        if (auto value = server.get_param<float>(key)) params[key] = *value;
    }
    params["launchcontroller/test_stage"] = server.get_param<int>("launchcontroller/test_stage").value_or(0);
    _handle_param_updates(params);
    // A true value in a saved config must never initiate a launch at startup.
    _enabled = server.get_param<bool>("launchcontroller/testing_enabled").value_or(false);
    _brake_requested = server.get_param<bool>("launchcontroller/brake_requested").value_or(false);
    _abort_requested = server.get_param<bool>("launchcontroller/abort_requested").value_or(false);
    _reset_emergency = server.get_param<bool>("launchcontroller/reset_emergency").value_or(false);
    _param_connection = server.register_param_callback(
        [this](const auto& updated) { _handle_param_updates(updated); });
    return _config_valid && _heading_config_valid;
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
    bool stage_changed = false;
    if (auto it = params.find("launchcontroller/test_stage"); it != params.end()) {
        const auto* value = std::get_if<int>(&it->second);
        const auto stage = value && *value >= 0 && *value <= 4 ? static_cast<TestStage>(*value) : TestStage::DISABLED;
        if (!value || *value < 0 || *value > 4)
            spdlog::error("LaunchController: test_stage must be an integer from 0 to 4; disabling");
        stage_changed = stage != _test_stage;
        _test_stage = stage;
    }
    update("heading_kp", _heading_config.heading.kp);
    update("heading_kd", _heading_config.heading.kd);
    update("max_steering_column_angle_deg", _heading_config.heading.max_column_angle_deg);
    update("steering_sign", _heading_config.heading.steering_sign);
    update("fake_heading_max_deg", _heading_config.fake_heading_max_deg);
    _heading_config_valid = HeadingHold::valid(_heading_config.heading) &&
        std::isfinite(_heading_config.fake_heading_max_deg) && _heading_config.fake_heading_max_deg > 0 &&
        _heading_config.fake_heading_max_deg <= 180;
    _start_requested = _start_requested || (enabled && !_enabled);
    const bool reset_emergency = read_bool("reset_emergency", _reset_emergency);
    _reset_emergency_requested = _reset_emergency_requested || (reset_emergency && !_reset_emergency);
    _reset_emergency = reset_emergency;
    _abort_requested = read_bool("abort_requested", _abort_requested);
    if (!enabled || stage_changed || _abort_requested) _remote_mission.abort();
    // Disabling or changing stage cancels the run immediately. Never carry a
    // pending start across stages; require a new false-to-true enable edge.
    if (!enabled || stage_changed || _test_stage == TestStage::DISABLED ||
        (_test_stage == TestStage::GROUND && _abort_requested)) {
        _start_requested = false;
        _steering_only_active = false;
        _profile = LaunchProfile{};
        _last_output = {core::SpeedControlOut{}, 0.0f, false};
        _last_step = {};
    }
    _enabled = enabled;
    _brake_requested = read_bool("brake_requested", _brake_requested);
}

void control::LaunchController::reset() {
    std::scoped_lock lock(_mutex);
    // Controller swaps cancel a remote mission but cannot clear Emergency.
    _remote_mission.abort();
    _profile = LaunchProfile{};
    _last_output = {core::SpeedControlOut{}, 0.0f, false};
    _start_requested = false;
    _reset_emergency_requested = false;
    _steering_only_active = false;
    _last_step = {};
}

core::ControllerOutput control::LaunchController::step_controller(const core::VehicleState& in) {
    std::scoped_lock lock(_mutex);
    const auto now = std::chrono::steady_clock::now();
    if (_reset_emergency_requested) {
        if (!_abort_requested) _remote_mission.reset(_enabled, in.rss, now);
        _reset_emergency_requested = false;
    }
    // Also expose the latched request while the operator has disabled testing.
    _log_remote_mission(now);
    if (!_enabled || _test_stage == TestStage::DISABLED) return _last_output;
    if (_test_stage == TestStage::XBOX_STEERING) {
        const auto target = _steering_target ? _steering_target() : std::nullopt;
        const bool connected = target && std::isfinite(*target);
        if (_start_requested) {
            _steering_only_active = connected;
            _start_requested = false;
        }
        if (!connected) _steering_only_active = false;
        // Reuse manual teleop steering without running any drive profile. No
        // RTD, wheel-speed or OMS data is needed; Kraken validates its feedback.
        _last_output = {core::SpeedControlOut{}, _steering_only_active ? *target : 0.0f, _steering_only_active};
        return _last_output;
    }
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
    const bool ground_stage = _test_stage == TestStage::GROUND;
    const bool xbox_stage = _test_stage == TestStage::XBOX_HEADING;
    const auto stick = xbox_stage && _steering_input ? _steering_input() : std::nullopt;
    if (xbox_stage) healthy = healthy && stick && std::isfinite(*stick);
    if (ground_stage) {
        const bool arm_requested = _start_requested;
        _start_requested = false;
        if (arm_requested) _remote_mission.arm(in.rss, now);
        if (_abort_requested) _remote_mission.abort();
        const auto feedback = _integration_feedback ? _integration_feedback() : RemoteMission::Feedback{};
        _start_requested = _remote_mission.update(in.rss, feedback, now);
        if (_remote_mission.state() != RemoteMission::State::RUNNING) {
            _profile = LaunchProfile{};
            _last_output = {core::SpeedControlOut{}, 0.0f, false};
            return _last_output;
        }
    }
    if (_start_requested) {
        if (previous_phase != LaunchProfile::Phase::ACCELERATING && previous_phase != LaunchProfile::Phase::BRAKING) {
            _active_heading_config = _heading_config;
            // The Xbox simulates an absolute heading error about centered stick.
            _heading_hold.reset(ground_stage ? in.oms.heading_deg : 0.0);
        }
    }
    if (ground_stage) {
        healthy = healthy && in.oms.received && std::isfinite(in.oms.heading_deg) && std::isfinite(in.oms.yaw_rate_deg_s);
    }
    const std::optional<LaunchProfile::GroundMotion> ground = ground_stage ?
        std::optional<LaunchProfile::GroundMotion>{{in.oms.speed_mps, in.oms.distance_m}} : std::nullopt;
    if (_start_requested) {
        _profile.start(_config, healthy && _config_valid && _heading_config_valid && !_brake_requested, speeds, ground);
        _start_requested = false;
        dt = 0.0f;
    }
    const auto result = _profile.step(dt, speeds, healthy,
        _brake_requested || in.input.requested_brake > 0.2f, ground);
    if (ground_stage && (_profile.phase() == LaunchProfile::Phase::FAULT ||
                         _profile.phase() == LaunchProfile::Phase::COMPLETE)) {
        if (_profile.phase() == LaunchProfile::Phase::FAULT) _remote_mission.abort();
        else _remote_mission.braking_complete();
        _last_output = {core::SpeedControlOut{}, 0.0f, false};
        return _last_output;
    }
    if (_profile.phase() != previous_phase) {
        spdlog::info("LaunchController: phase={} distance={}m deceleration={}m/s^2",
            static_cast<int>(_profile.phase()), _profile.distance_m(), _profile.deceleration_mps2());
    }
    const float rpm = result.speed_mps * constants::METERS_PER_SECOND_TO_RPM;
    const float torque = result.torque_limit_nm;
    const bool running = _profile.phase() == LaunchProfile::Phase::ACCELERATING || _profile.phase() == LaunchProfile::Phase::BRAKING;
    // Linux joydev reports left negative. HeadingHold uses left-positive heading;
    // no simulated yaw rate is added, so holding the stick holds a heading error.
    const double heading = xbox_stage && stick ?
        -std::clamp(*stick, -1.0f, 1.0f) * _active_heading_config.fake_heading_max_deg : in.oms.heading_deg;
    const float steering = running && (ground_stage || xbox_stage) ?
        _heading_hold.step(_active_heading_config.heading, heading, ground_stage ? in.oms.yaw_rate_deg_s : 0.0f) : 0.0f;
    _last_output = {core::SpeedControlOut{in.prev_MCU_recv_millis,
        {rpm, rpm, rpm, rpm}, {torque, torque, torque, torque}}, steering, running};
    return _last_output;
}

void control::LaunchController::_log_remote_mission(std::chrono::steady_clock::time_point now) {
    if (_last_mission_telem.time_since_epoch().count() && now - _last_mission_telem < std::chrono::milliseconds(100)) return;
    _last_mission_telem = now;
    auto msg = std::make_shared<google::protobuf::Struct>();
    auto& fields = *msg->mutable_fields();
    const auto requests = _remote_mission.requests();
    fields["state"].set_number_value(static_cast<int>(_remote_mission.state()));
    fields["start_requested"].set_bool_value(requests.start);
    fields["start_request_count"].set_number_value(_remote_mission.start_requests());
    fields["emergency_requested"].set_bool_value(requests.emergency);
    fields["finished_requested"].set_bool_value(requests.finished);
    fields["emergency_ack"].set_bool_value(_remote_mission.emergency_ack());
    fields["counterpart_connected"].set_bool_value(static_cast<bool>(_integration_feedback));
    core::log_mcap_only(msg, "launchcontroller/remote_mission");
    core::log_foxglove_only(msg, "launchcontroller/remote_mission");
}
