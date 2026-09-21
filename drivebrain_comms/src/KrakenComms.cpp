#include <KrakenComms.hpp>

#include <algorithm>
#include <chrono>

#include <spdlog/spdlog.h>

#include "FoxgloveServer.hpp"
#include "StateTracker.hpp"

namespace comms
{
    KrakenComms::KrakenComms() {}

    KrakenComms::~KrakenComms()
    {
        _running = false;
        if (_thread.joinable())
        {
            _thread.join();
        }
    }

    bool KrakenComms::init()
    {
        auto& foxglove = core::FoxgloveServer::instance();

        auto device_id = foxglove.get_param<int>("kraken_comms/device_id");
        auto canbus_name = foxglove.get_param<std::string>("kraken_comms/canbus_name");
        auto send_rate_hz = foxglove.get_param<int>("kraken_comms/send_rate_hz");
        auto reduction = foxglove.get_param<float>("kraken_comms/reduction");
        auto min_angle_deg = foxglove.get_param<float>("kraken_comms/min_angle_deg");
        auto max_angle_deg = foxglove.get_param<float>("kraken_comms/max_angle_deg");

        if (!(device_id && canbus_name && send_rate_hz && reduction && min_angle_deg && max_angle_deg))
        {
            spdlog::error("Couldn't load all params for KrakenComms");
            return false;
        }

        _config.device_id = device_id.value();
        _config.canbus_name = canbus_name.value();
        _config.send_rate_hz = send_rate_hz.value();
        _config.reduction = reduction.value();
        _config.min_angle_deg = min_angle_deg.value();
        _config.max_angle_deg = max_angle_deg.value();

        if (_config.send_rate_hz <= 0)
        {
            spdlog::error("KrakenComms: send_rate_hz must be > 0, got {}", _config.send_rate_hz);
            return false;
        }

        _kraken = std::make_unique<ctre::phoenix6::hardware::TalonFX>(_config.device_id, _config.canbus_name);

        ctre::phoenix6::configs::TalonFXConfiguration talon_config{};
        talon_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        // TODO: tune Slot0 gains and CurrentLimits before running closed loop on hardware

        const double forward_limit_rotations = (static_cast<double>(_config.max_angle_deg) / 360.0) * _config.reduction;
        const double reverse_limit_rotations = (static_cast<double>(_config.min_angle_deg) / 360.0) * _config.reduction;
        talon_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talon_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = units::angle::turn_t{forward_limit_rotations};
        talon_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talon_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = units::angle::turn_t{reverse_limit_rotations};

        auto status = _kraken->GetConfigurator().Apply(talon_config);
        if (!status.IsOK())
        {
            spdlog::error("KrakenComms: failed to apply TalonFX config: {}", status.GetName());
            return false;
        }

        auto state_and_validity = core::StateTracker::instance().get_latest_state_and_validity();
        if (!state_and_validity.second)
        {
            spdlog::warn("KrakenComms::init() - no valid steering sensor reading yet, seeding angle to 0");
            _angle_deg = 0.0f;
        }
        else
        {
            _angle_deg = std::clamp(state_and_validity.first.steering_angle_deg,
                                     _config.min_angle_deg, _config.max_angle_deg);
        }

        _running = true;
        _thread = std::thread([this]() {
            try
            {
                _run();
            }
            catch (const std::exception& e)
            {
                spdlog::error("KrakenComms thread threw: {}", e.what());
            }
            catch (...)
            {
                spdlog::error("KrakenComms thread threw unknown exception");
            }
            spdlog::error("KrakenComms thread exiting, running={}", _running.load());
        });

        return true;
    }

    void KrakenComms::set_angle(float angle_deg)
    {
        std::scoped_lock lock(_angle_mutex);
        _angle_deg = std::clamp(angle_deg, _config.min_angle_deg, _config.max_angle_deg);
    }

    float KrakenComms::get_commanded_angle_deg() const
    {
        std::scoped_lock lock(_angle_mutex);
        return _angle_deg;
    }

    float KrakenComms::get_measured_angle_deg()
    {
        double rotations = _kraken->GetPosition().GetValueAsDouble();
        return static_cast<float>(rotations * 360.0 / _config.reduction);
    }

    void KrakenComms::_run()
    {
        using namespace std::chrono;

        const auto period = microseconds(static_cast<long long>(1'000'000.0 / _config.send_rate_hz));
        auto next_tick = steady_clock::now();

        ctre::phoenix6::controls::PositionVoltage request{units::angle::turn_t{0.0}};

        while (_running)
        {
            next_tick += period;

            float target_deg;
            {
                std::scoped_lock lock(_angle_mutex);
                target_deg = _angle_deg;
            }

            const double target_rotations = (static_cast<double>(target_deg) / 360.0) * _config.reduction;
            _kraken->SetControl(request.WithPosition(units::angle::turn_t{target_rotations}));

            auto now = steady_clock::now();
            if (now > next_tick)
            {
                spdlog::warn("KrakenComms loop overrun by {}us", duration_cast<microseconds>(now - next_tick).count());
                next_tick = now;
            }
            std::this_thread::sleep_until(next_tick);
        }
    }
}
