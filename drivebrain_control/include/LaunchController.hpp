#pragma once

#include <Controller.hpp>
#include <FoxgloveServer.hpp>
#include <StateTracker.hpp>
#include <LaunchProfile.hpp>
#include <HeadingHold.hpp>
#include <RemoteMission.hpp>
#include <chrono>
#include <functional>
#include <mutex>

namespace control {
class LaunchController : public Controller<core::ControllerOutput, core::VehicleState> {
public:
    // Normalized Xbox steering axis: left=-1, right=+1; no value when disconnected.
    using SteeringInput = std::function<std::optional<float>()>;
    // In-process simulation seam only. The app supplies no counterpart until the
    // actual RDC/EBS interface is confirmed; default feedback denies launch.
    using IntegrationFeedback = std::function<RemoteMission::Feedback()>;
    explicit LaunchController(SteeringInput steering_input = {}, SteeringInput steering_target = {},
                              IntegrationFeedback integration_feedback = {})
        : _steering_input(std::move(steering_input)), _steering_target(std::move(steering_target)),
          _integration_feedback(std::move(integration_feedback)) {}
    RemoteMission remote_mission() {
        std::scoped_lock lock(_mutex);
        return _remote_mission;
    }
    bool init();
    float get_dt_sec() override { return 0.004f; }
    core::ControllerOutput step_controller(const core::VehicleState& in) override;
    core::ControllerOutput preview_controller(const core::VehicleState&) override {
        std::scoped_lock lock(_mutex);
        return _last_output;
    }
    void reset() override;

private:
    void _handle_param_updates(const std::unordered_map<std::string, core::DBParam>& params);
    std::mutex _mutex;
    LaunchProfile::Config _config{};
    LaunchProfile _profile;
    enum class TestStage { DISABLED = 0, XBOX_STEERING = 1, LINEAR_RAMP = 2, XBOX_HEADING = 3, GROUND = 4 };
    struct HeadingConfig {
        HeadingHold::Config heading{};
        float fake_heading_max_deg = 10.0f;
    };
    TestStage _test_stage = TestStage::DISABLED;
    HeadingConfig _heading_config{};
    HeadingConfig _active_heading_config{};
    SteeringInput _steering_input;
    SteeringInput _steering_target;
    IntegrationFeedback _integration_feedback;
    RemoteMission _remote_mission;
    bool _reset_emergency = false;
    bool _reset_emergency_requested = false;
    void _log_remote_mission(std::chrono::steady_clock::time_point now);
    std::chrono::steady_clock::time_point _last_mission_telem{};
    bool _steering_only_active = false;
    HeadingHold _heading_hold;
    core::ControllerOutput _last_output{core::SpeedControlOut{}, 0.0f, false};
    bool _enabled = false;
    bool _brake_requested = false;
    bool _abort_requested = false;
    bool _start_requested = false;
    bool _config_valid = false;
    bool _heading_config_valid = true;
    std::chrono::steady_clock::time_point _last_step{};
    boost::signals2::scoped_connection _param_connection;
};
}
