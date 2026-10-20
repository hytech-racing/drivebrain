#pragma once

#include <Controller.hpp>
#include <StateTracker.hpp>
#include <Literals.hpp>
#include <FoxgloveServer.hpp>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

using namespace core;

namespace control
{
    class TeleopController : public Controller<core::ControllerOutput, core::VehicleState> {
    public:

        struct config
        {
            torque_nm max_torque;
            torque_nm max_regen_torque;
            float max_steering_angle_deg;
            int dt_rate_hz;

            std::string joystick_device_path; // e.g. "/dev/input/js0"
            int accel_axis;
            int brake_axis;
            int steering_axis;
            bool invert_accel_axis;
            bool invert_brake_axis;
            bool invert_steering_axis;
            float axis_deadzone;
        };

        TeleopController() = default;
        ~TeleopController();

        TeleopController(const TeleopController&) = delete;
        TeleopController& operator=(const TeleopController&) = delete;

        float get_dt_sec() override {
            return (double) 1.0 / _config.dt_rate_hz;
        }

        bool init();

        core::ControllerOutput step_controller(const VehicleState &in) override;

        // Available even when TeleopController is not selected; left=-1, right=+1.
        std::optional<float> get_steering_input() const {
            if (_fd < 0) return std::nullopt;
            return _steering.load();
        }
        // Original cubic Xbox steering response, in column degrees.
        std::optional<float> get_steering_column_target();

        std::atomic<float> steering_command_deg{0.0f};

    private:

        void _handle_param_updates(const std::unordered_map<std::string, DBParam> &new_param_map);

        void _run_joystick();

        /** @return raw stick value mapped to [-1, 1] */
        static float _normalize_axis(int16_t raw, bool invert, float deadzone);

        /** @return raw trigger value mapped to [0, 1], 0 at rest */
        static float _normalize_trigger(int16_t raw, bool invert, float deadzone);

        /** Zeroes all inputs and disarms the triggers, e.g. after losing the device */
        void _reset_inputs();

        std::mutex _config_mutex;
        config _config{};

        std::atomic<int> _fd{-1};
        std::thread _joystick_thread;
        std::atomic<bool> _running{false};

        std::atomic<float> _accel{0.0f};
        std::atomic<float> _brake{0.0f};
        std::atomic<float> _steering{0.0f};
        std::atomic<bool> _accel_armed{false};
        std::atomic<bool> _brake_armed{false};
    };
}
