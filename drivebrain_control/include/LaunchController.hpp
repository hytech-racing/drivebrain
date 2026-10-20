#pragma once

#include <Controller.hpp>
#include <FoxgloveServer.hpp>
#include <StateTracker.hpp>
#include <LaunchProfile.hpp>
#include <HeadingHold.hpp>
#include <chrono>
#include <mutex>

namespace control {
class LaunchController : public Controller<core::ControllerOutput, core::VehicleState> {
public:
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
    struct GroundConfig {
        bool use_ground_speed = true;
        bool heading_control_enabled = true;
        HeadingHold::Config heading{};
    };
    GroundConfig _ground_config{};
    GroundConfig _active_ground_config{};
    HeadingHold _heading_hold;
    core::ControllerOutput _last_output{core::SpeedControlOut{}, 0.0f};
    bool _enabled = false;
    bool _brake_requested = false;
    bool _start_requested = false;
    bool _config_valid = false;
    bool _ground_config_valid = true;
    std::chrono::steady_clock::time_point _last_step{};
    boost::signals2::scoped_connection _param_connection;
};
}
