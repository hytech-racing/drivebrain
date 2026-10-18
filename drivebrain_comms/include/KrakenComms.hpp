#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <memory>
#include <unordered_map>

#include <boost/signals2/connection.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/VoltageOut.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include "FoxgloveServer.hpp"

namespace comms
{
    /**
     * Steering position control runs on drivebrain. Each tick a trapezoidal motion profile walks a reference
     * toward the target angle, then a PID on the steering column angle (from the steering estimator) plus
     * S/V/A feedforward tracks that reference. The resulting voltage is streamed to the Kraken's TalonFX as an
     * open loop VoltageOut. The D term uses the Kraken's velocity, which is far less noisy than differentiating
     * the analog column angle. The motor encoder is seeded from the column angle once at startup so the
     * TalonFX soft limits line up with the column angle.
    */
    class KrakenComms
    {
    public:

        /**
         * @struct config_s
        */
        struct config_s
        {
            int device_id;
            std::string canbus_name;
            int send_rate_hz;
            float reduction;
            float min_angle_deg;
            float max_angle_deg;
            float stator_current_limit_a;
            float supply_current_limit_a;
            int enable_timeout_ms;
            /* Column angle older than this is treated as lost feedback and the motor is set to neutral */
            int steering_timeout_ms;
        };

        /**
         * @struct tuningParams_s
        */
        struct tuningParams_s
        {
            // Same units as the TalonFX Slot0 gains (mechanism rotations), so old Kraken tuning carries over

            // Feedback gains: PID on error between the profile reference and the column angle
            float kraken_P;
            float kraken_I;
            float kraken_D;

            // Feedforward gains: SVA. S is applied in the direction the profile is moving. Once the profile
            // has settled it follows the error instead, and is off inside the deadband.
            float kraken_S;
            float kraken_S_deadband_deg;
            float kraken_V;
            float kraken_A;

            // Trapezoidal motion profile limits
            float motion_magic_cruise_velocity_deg_s;
            float motion_magic_acceleration_deg_s2;

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

        /**
         * Online trapezoidal motion profile. Each step moves the reference toward the goal without exceeding
         * the cruise velocity or acceleration, braking early enough to stop on the goal. The goal can change
         * at any time, including mid-move.
        */
        class TrapezoidalProfile
        {
        public:
            struct state_s
            {
                float pos_deg;
                float vel_deg_s;
                float acc_deg_s2;
            };

            /**
             * @param goal_deg angle the profile should end up at
             * @param dt_s time since the last step
             * @return the new reference
            */
            state_s step(float goal_deg, float max_vel_deg_s, float max_acc_deg_s2, float dt_s);

            /**
             * Restarts the profile from the given state, e.g. where the column currently is.
            */
            void reset(float pos_deg, float vel_deg_s);

        private:
            state_s _state{};
        };

        /**
         * Stateful PID on column angle. Kept separate from the CAN code so it can be reasoned about alone.
        */
        class ColumnAnglePID
        {
        public:
            /**
             * @param reference profile state to track
             * @param measured_deg column angle from the steering estimator
             * @param measured_velocity_deg_s column angular velocity, used for the D term
             * @param dt_s time since the last step
             * @return voltage to apply
            */
            float step(const tuningParams_s &tuning, const TrapezoidalProfile::state_s &reference,
                       float measured_deg, float measured_velocity_deg_s, float dt_s);

            void reset();

            float get_last_integral_v() const { return _integral_v; }

        private:
            float _integral_v{0.0f};
        };

    private:

        /**
         * Zeroes the motor encoder once at startup (wheel assumed centered), then sends the current
         * angle target to the Kraken at config.send_rate_hz.
        */
        void _run();

        /**
         * Picks up live changes to the test override and tuning params from Foxglove.
         */
        void _handle_param_updates(const std::unordered_map<std::string, core::DBParam> &new_param_map);

        static units::angle::turn_t _deg_to_turns(double degree);

        /**
         * Checks the PID limits are usable, logging why if not.
         * @return true if cruise velocity and acceleration are > 0 and the kS deadband is >= 0
        */
        static bool _is_tuning_valid(const tuningParams_s &tuning_params);

        /**
         * @param sticky read sticky faults instead of currently active ones
         * @return comma-separated names of the motor's faults, or "none"
        */
        std::string _describe_faults(bool sticky);

        config_s _config{};

        std::mutex _tuning_mutex;
        tuningParams_s _tuning_params{};

        /* When false, the motor is held in neutral */
        std::atomic<bool> _test_mode{false};
        /* When true (and test mode is on), _run() targets _test_angle_deg instead of set_angle() input */
        std::atomic<bool> _use_test_angle{false};
        std::atomic<float> _test_angle_deg{0.0f};
        /* When true (and test mode is on), _run() targets a hard coded sinusoid, overriding the other inputs */
        std::atomic<bool> _use_sine_test{false};
        /* Multiplier on the sine test frequency, 1.0 is the base 4 s period */
        std::atomic<float> _sine_speed_scale{1.0f};
        boost::signals2::scoped_connection _param_connection;

        std::unique_ptr<ctre::phoenix6::hardware::TalonFX> _kraken;

        mutable std::mutex _angle_mutex;
        float _angle_deg{0.0f};

        std::atomic<bool> _running{false};
        std::thread _thread;
    };
}
