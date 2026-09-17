#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <memory>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>

namespace comms
{
    class KrakenComms
    {
    public:
        struct config
        {
            int device_id;
            std::string canbus_name;
            int send_rate_hz;
            float gear_ratio;
            float min_angle_deg;
            float max_angle_deg;
        };

        KrakenComms();
        ~KrakenComms();

        KrakenComms(const KrakenComms&) = delete;
        KrakenComms& operator=(const KrakenComms&) = delete;

        /**
         * Initializes the Kraken CAN connection, loads config, and starts the
         * background command thread.
         *
         * @return true if initialization succeeded
         */
        bool init();

        /**
         * Thread-safe setter for the desired steering angle.
         *
         * @param angle_deg desired steering angle in degrees
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
         * Sends the current angle target to the Kraken at config.send_rate_hz.
         * Runs on _thread for the lifetime of the object.
         */
        void _run();

        config _config{};

        std::unique_ptr<ctre::phoenix6::hardware::TalonFX> _kraken;

        mutable std::mutex _angle_mutex;
        float _angle_deg{0.0f};

        std::atomic<bool> _running{false};
        std::thread _thread;
    };
}
