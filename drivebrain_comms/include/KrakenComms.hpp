#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <memory>
#include <unordered_map>

#include <boost/signals2/connection.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include "FoxgloveServer.hpp"

namespace comms
{
    /**
     * All closed-loop control runs on the Kraken's TalonFX. Drivebrain only seeds the motor's encoder
     * from the steering sensor, streams a target angle, and feeds the enable watchdog.
    */
    class KrakenComms
    {
    public:

        /**
         * Static configuration for the steering motor.
         * Loaded once from Foxglove params in init() and not changed afterwards
         * (live-updatable gains live in the separate `tuningParams` struct).
         *
         * Angles are in steering-shaft degrees, i.e. after the gear reduction.
        */
        struct config_s
        {
            int device_id;
            std::string canbus_name;        // This should be the SocketCAN interface the motor is on (e.g. "can0" or "can1").
            int send_rate_hz;               // Rate (Hz) at which _run() feeds the enable watchdog and sends the angle target.
            float reduction;
            float min_angle_deg;
            float max_angle_deg;
            float stator_current_limit_a;
            int enable_timeout_ms;          // Watchdog timeout (ms) passed to FeedEnable. The motor disables itself if not fed within this window.
            float max_disagreement_deg;     // Max allowed difference between motor and steering sensor angles before faulting.
        };

        /**
         * CTRE's onboard controller tuning, updatable from Foxglove.
        */
        struct tuningParams_s
        {
            // Feedback gains: PID
            float kraken_P;     // V/rot:       output per unit of position error
            float kraken_I;     // V/(rot·s):   output per unit of integrated position error
            float kraken_D;     // V/(rot/s):   output per unit of velocity error

            // Feedforward gains: SVA
            float kraken_S;     // V:           output to overcome static friction
            float kraken_V;     // V/(rot/s):   output per unit of target velocity
            float kraken_A;     // V/(rot/s²):  output per unit of target acceleration

            float motion_magic_cruise_velocity_deg_s;
            float motion_magic_acceleration_deg_s2;
            float motion_magic_jerk_deg_s3;

            bool operator==(const tuningParams_s&) const = default;
        };

        KrakenComms();
        ~KrakenComms();

        KrakenComms(const KrakenComms&) = delete;
        KrakenComms& operator=(const KrakenComms&) = delete;

        /**
         * Initializes the Kraken CAN connection, loads config, and starts the background command thread.
         * @return true if initialization succeeded
        */
        bool init();

        /**
         * Setter for the desired steering angle.
         * @param angle_deg desired steering angle in degrees.
        */
        void set_angle(float angle_deg);

        /**
         * @return the currently commanded angle in degrees
        */
        float get_commanded_angle_deg() const;

        /**
         * @return the Kraken's measured angle in degrees
        */
        float get_measured_angle_deg();

    private:

        /**
         * Seeds the motor encoder once the steering sensor is valid, then sends the current
         * angle target to the Kraken at config.send_rate_hz.
        */
        void _run();

        /**
         * Picks up live changes to the test override and tuning params from Foxglove.
         */
        void _handle_param_updates(const std::unordered_map<std::string, core::DBParam> &new_param_map);

        static ctre::phoenix6::configs::Slot0Configs _make_slot0_configs(const tuningParams_s &tuning_params);

        static ctre::phoenix6::configs::MotionMagicConfigs _make_motion_magic_configs(const tuningParams_s &tuning_params);

        /**
         * Pushes Slot0 and Motion Magic configs to the motor
         * @return true if both configs applied, false otherwise
        */
        bool _apply_tuning(const tuningParams_s &new_tuning);

        static units::angle::turn_t _deg_to_turns(double degree);

        /**
         * Checks Motion Magic limits can actually move the motor, logging why if not.
         * @return true if cruise velocity and acceleration are > 0 and jerk is >= 0
        */
        static bool _is_motion_magic_valid(const tuningParams_s &tuning_params);

        config_s _config{};

        std::mutex _tuning_mutex;
        tuningParams_s _tuning_params{};

        /* When true, _run() targets _test_angle_deg instead of set_angle() input */
        std::atomic<bool> _test_mode{false};
        std::atomic<float> _test_angle_deg{0.0f};
        boost::signals2::scoped_connection _param_connection;

        std::unique_ptr<ctre::phoenix6::hardware::TalonFX> _kraken;

        mutable std::mutex _angle_mutex;
        float _angle_deg{0.0f};

        std::atomic<bool> _running{false};
        std::thread _thread;
    };
}
