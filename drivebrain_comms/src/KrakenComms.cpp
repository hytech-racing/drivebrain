#include <KrakenComms.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>

#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>
#include <spdlog/spdlog.h>

#include "StateTracker.hpp"


namespace comms
{
    KrakenComms::KrakenComms() {}

    KrakenComms::~KrakenComms()
    {
        _param_connection.disconnect();
        _running = false;
        if (_thread.joinable())
        {
            _thread.join();
        }
    }

    bool KrakenComms::init()
    {
        auto &foxglove = core::FoxgloveServer::instance();

        auto device_id = foxglove.get_param<int>("kraken_comms/device_id");
        auto canbus_name = foxglove.get_param<std::string>("kraken_comms/canbus_name");
        auto send_rate_hz = foxglove.get_param<int>("kraken_comms/send_rate_hz");
        auto reduction = foxglove.get_param<float>("kraken_comms/reduction");
        auto min_angle_deg = foxglove.get_param<float>("kraken_comms/min_angle_deg");
        auto max_angle_deg = foxglove.get_param<float>("kraken_comms/max_angle_deg");
        auto stator_current_limit_a = foxglove.get_param<float>("kraken_comms/stator_current_limit_a");
        auto enable_timeout_ms = foxglove.get_param<int>("kraken_comms/enable_timeout_ms");
        auto max_disagreement_deg = foxglove.get_param<float>("kraken_comms/max_disagreement_deg");

        auto kraken_P = foxglove.get_param<float>("kraken_comms/kp");
        auto kraken_I = foxglove.get_param<float>("kraken_comms/ki");
        auto kraken_D = foxglove.get_param<float>("kraken_comms/kd");
        auto kraken_S = foxglove.get_param<float>("kraken_comms/ks");
        auto kraken_V = foxglove.get_param<float>("kraken_comms/kv");
        auto kraken_A = foxglove.get_param<float>("kraken_comms/ka");
        auto motion_magic_cruise_velocity_deg_s = foxglove.get_param<float>("kraken_comms/mm_cruise_velocity_deg_s");
        auto motion_magic_acceleration_deg_s2 = foxglove.get_param<float>("kraken_comms/mm_acceleration_deg_s2");
        auto motion_magic_jerk_deg_s3 = foxglove.get_param<float>("kraken_comms/mm_jerk_deg_s3");

        if (!(device_id && canbus_name && send_rate_hz && reduction && min_angle_deg && max_angle_deg &&
              stator_current_limit_a && enable_timeout_ms && max_disagreement_deg &&
              kraken_P && kraken_I && kraken_D && kraken_S && kraken_V && kraken_A &&
              motion_magic_cruise_velocity_deg_s && motion_magic_acceleration_deg_s2 && motion_magic_jerk_deg_s3))
        {
            spdlog::error("Couldn't load all params for KrakenComms");
            return false;
        }

        _config = {
            .device_id = device_id.value(),
            .canbus_name = canbus_name.value(),
            .send_rate_hz = send_rate_hz.value(),
            .reduction = reduction.value(),
            .min_angle_deg = min_angle_deg.value(),
            .max_angle_deg = max_angle_deg.value(),
            .stator_current_limit_a = stator_current_limit_a.value(),
            .enable_timeout_ms = enable_timeout_ms.value(),
            .max_disagreement_deg = max_disagreement_deg.value(),
        };

        _tuning_params = {
            .kraken_P = kraken_P.value(),
            .kraken_I = kraken_I.value(),
            .kraken_D = kraken_D.value(),
            .kraken_S = kraken_S.value(),
            .kraken_V = kraken_V.value(),
            .kraken_A = kraken_A.value(),
            .motion_magic_cruise_velocity_deg_s = motion_magic_cruise_velocity_deg_s.value(),
            .motion_magic_acceleration_deg_s2 = motion_magic_acceleration_deg_s2.value(),
            .motion_magic_jerk_deg_s3 = motion_magic_jerk_deg_s3.value(),
        };

        if (_config.send_rate_hz <= 0)
        {
            spdlog::error("KrakenComms: send_rate_hz must be > 0, got {}", _config.send_rate_hz);
            return false;
        }

        // std::clamp is undefined behavior if min > max
        if (_config.min_angle_deg >= _config.max_angle_deg)
        {
            spdlog::error("KrakenComms: min_angle_deg ({}) must be < max_angle_deg ({})",
                          _config.min_angle_deg, _config.max_angle_deg);
            return false;
        }

        if (_config.reduction <= 0.0f)
        {
            spdlog::error("KrakenComms: reduction must be > 0, got {}", _config.reduction);
            return false;
        }

        // A timeout near the send period lets the motor drop out between ticks
        const int min_enable_timeout_ms = 3 * 1000 / _config.send_rate_hz;
        if (_config.enable_timeout_ms < min_enable_timeout_ms)
        {
            spdlog::error("KrakenComms: enable_timeout_ms must be >= {} (3x the send period), got {}",
                          min_enable_timeout_ms, _config.enable_timeout_ms);
            return false;
        }

        if (_config.max_disagreement_deg <= 0.0f)
        {
            spdlog::error("KrakenComms: max_disagreement_deg must be > 0, got {}", _config.max_disagreement_deg);
            return false;
        }

        if (!_is_motion_magic_valid(_tuning_params))
        {
            return false;
        }

        _kraken = std::make_unique<ctre::phoenix6::hardware::TalonFX>(_config.device_id, _config.canbus_name);

        ctre::phoenix6::configs::TalonFXConfiguration talon_config{};
        talon_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
        /**
         * Convention: clockwise steering-wheel rotation, as seen by the driver, is positive.
         * CTRE defines clockwise looking at the motor's shaft face. The motor's shaft faces the
         * driver and drives the wheel directly, so the two views match.
        */
        talon_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

        // Motor reports and accepts positions in steering-shaft rotations
        talon_config.Feedback.SensorToMechanismRatio = units::dimensionless::scalar_t{_config.reduction};

        talon_config.Slot0 = _make_slot0_configs(_tuning_params);
        talon_config.MotionMagic = _make_motion_magic_configs(_tuning_params);

        talon_config.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{_config.stator_current_limit_a};
        talon_config.CurrentLimits.StatorCurrentLimitEnable = true;

        talon_config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        talon_config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = _deg_to_turns(_config.max_angle_deg);
        talon_config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        talon_config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = _deg_to_turns(_config.min_angle_deg);

        auto status = _kraken->GetConfigurator().Apply(talon_config);
        if (!status.IsOK())
        {
            spdlog::error("KrakenComms: failed to apply TalonFX config: {}", status.GetName());
            return false;
        }

        // Test override; missing params just leave test mode off
        _test_mode = foxglove.get_param<bool>("kraken_comms/test_mode").value_or(false);
        _test_angle_deg = foxglove.get_param<float>("kraken_comms/test_angle_deg").value_or(0.0f);
        if (_test_mode)
        {
            spdlog::warn("KrakenComms: test_mode enabled, targeting {} deg", _test_angle_deg.load());
        }

        _param_connection = foxglove.register_param_callback(
            [this](const std::unordered_map<std::string, core::DBParam> &params) {
                _handle_param_updates(params);
            });

        _running = true;
        _thread = std::thread([this]() {
            try
            {
                _run();
            }
            catch (const std::exception &e)
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
        if (!_kraken)
        {
            return 0.0f;
        }
        return static_cast<float>(_kraken->GetPosition().GetValueAsDouble() * 360.0);
    }

    void KrakenComms::_handle_param_updates(const std::unordered_map<std::string, core::DBParam> &new_param_map)
    {
        if (auto is_test_mode_on = core::process_param_update<bool>(new_param_map, "kraken_comms/test_mode"))
        {
            if (*is_test_mode_on != _test_mode.load())
            {
                spdlog::warn("KrakenComms: test_mode {}", *is_test_mode_on ? "enabled" : "disabled");
            }
            _test_mode = *is_test_mode_on;
        }

        if (auto new_test_angle = core::process_param_update<float>(new_param_map, "kraken_comms/test_angle_deg"))
        {
            _test_angle_deg = *new_test_angle;
        }

        tuningParams_s new_tuning{};
        {
            std::scoped_lock lock(_tuning_mutex);
            new_tuning = _tuning_params;
        }

        auto update = [&](const char *key, float &field) {
            if (auto v = core::process_param_update<float>(new_param_map, key))
            {
                field = *v;
            }
        };
        update("kraken_comms/kp", new_tuning.kraken_P);
        update("kraken_comms/ki", new_tuning.kraken_I);
        update("kraken_comms/kd", new_tuning.kraken_D);
        update("kraken_comms/ks", new_tuning.kraken_S);
        update("kraken_comms/kv", new_tuning.kraken_V);
        update("kraken_comms/ka", new_tuning.kraken_A);
        update("kraken_comms/mm_cruise_velocity_deg_s", new_tuning.motion_magic_cruise_velocity_deg_s);
        update("kraken_comms/mm_acceleration_deg_s2", new_tuning.motion_magic_acceleration_deg_s2);
        update("kraken_comms/mm_jerk_deg_s3", new_tuning.motion_magic_jerk_deg_s3);

        bool changed;
        {
            std::scoped_lock lock(_tuning_mutex);
            changed = !(new_tuning == _tuning_params);
        }
        if (changed && _is_motion_magic_valid(new_tuning))
        {
            _apply_tuning(new_tuning);
        }
    }

    bool KrakenComms::_is_motion_magic_valid(const tuningParams_s &tuning_params)
    {
        // Zero or negative velocity/acceleration leaves Motion Magic unable to move the motor.
        // Jerk of 0 is valid and means no jerk limit.
        if (tuning_params.motion_magic_cruise_velocity_deg_s <= 0.0f ||
            tuning_params.motion_magic_acceleration_deg_s2 <= 0.0f ||
            tuning_params.motion_magic_jerk_deg_s3 < 0.0f)
        {
            spdlog::error("KrakenComms: rejected Motion Magic limits cruise={}deg/s accel={}deg/s^2 jerk={}deg/s^3 "
                          "(cruise and accel must be > 0, jerk >= 0)",
                          tuning_params.motion_magic_cruise_velocity_deg_s,
                          tuning_params.motion_magic_acceleration_deg_s2,
                          tuning_params.motion_magic_jerk_deg_s3);
            return false;
        }
        return true;
    }

    ctre::phoenix6::configs::Slot0Configs KrakenComms::_make_slot0_configs(const tuningParams_s &tuning_params)
    {
        ctre::phoenix6::configs::Slot0Configs slot0{};
        slot0.kP = units::dimensionless::scalar_t{tuning_params.kraken_P};
        slot0.kI = units::dimensionless::scalar_t{tuning_params.kraken_I};
        slot0.kD = units::dimensionless::scalar_t{tuning_params.kraken_D};
        slot0.kS = units::dimensionless::scalar_t{tuning_params.kraken_S};
        slot0.kV = units::dimensionless::scalar_t{tuning_params.kraken_V};
        slot0.kA = units::dimensionless::scalar_t{tuning_params.kraken_A};
        return slot0;
    }

    ctre::phoenix6::configs::MotionMagicConfigs KrakenComms::_make_motion_magic_configs(const tuningParams_s &tuning_params)
    {
        ctre::phoenix6::configs::MotionMagicConfigs motion_magic{};
        motion_magic.MotionMagicCruiseVelocity =
            units::angular_velocity::turns_per_second_t{tuning_params.motion_magic_cruise_velocity_deg_s / 360.0};
        motion_magic.MotionMagicAcceleration =
            units::angular_acceleration::turns_per_second_squared_t{tuning_params.motion_magic_acceleration_deg_s2 / 360.0};
        motion_magic.MotionMagicJerk =
            units::angular_jerk::turns_per_second_cubed_t{tuning_params.motion_magic_jerk_deg_s3 / 360.0};
        return motion_magic;
    }

    units::angle::turn_t KrakenComms::_deg_to_turns(double degree)
    {
        return units::angle::turn_t{degree / 360.0};
    }

    bool KrakenComms::_apply_tuning(const tuningParams_s &new_tuning_params)
    {
        if (!_kraken)
        {
            return false;
        }

        auto &configurator = _kraken->GetConfigurator();

        // Apply both groups before checking, so one failure doesn't skip the other
        const auto slot0_status = configurator.Apply(_make_slot0_configs(new_tuning_params));
        const auto motion_magic_status = configurator.Apply(_make_motion_magic_configs(new_tuning_params));

        if (!slot0_status.IsOK() || !motion_magic_status.IsOK())
        {
            // The motor may now hold a mix of old and new values. _tuning_params is left unchanged, so the next Foxglove param update
            // (of any param) still sees a difference and retries the apply.
            // Might want to handle this case better, what if no params are updated?
            spdlog::error("KrakenComms: failed to apply tuning (slot0: {}, motion magic: {})",
                          slot0_status.GetName(), motion_magic_status.GetName());
            return false;
        }

        {
            std::scoped_lock lock(_tuning_mutex);
            _tuning_params = new_tuning_params;
        }

        spdlog::info("KrakenComms: applied tuning kP={} kI={} kD={} kS={} kV={} kA={} "
                     "cruise={}deg/s accel={}deg/s^2 jerk={}deg/s^3",
                     new_tuning_params.kraken_P, new_tuning_params.kraken_I, new_tuning_params.kraken_D,
                     new_tuning_params.kraken_S, new_tuning_params.kraken_V, new_tuning_params.kraken_A,
                     new_tuning_params.motion_magic_cruise_velocity_deg_s,
                     new_tuning_params.motion_magic_acceleration_deg_s2,
                     new_tuning_params.motion_magic_jerk_deg_s3);
        return true;
    }

    void KrakenComms::_run()
    {
        using namespace std::chrono;

        const auto period = microseconds(static_cast<long long>(1'000'000.0 / _config.send_rate_hz));
        auto next_tick = steady_clock::now();

        // FOC needs a Phoenix Pro license, so stick to standard commutation
        ctre::phoenix6::controls::MotionMagicVoltage request{units::angle::turn_t{0.0}};
        request.WithEnableFOC(false);
        ctre::phoenix6::controls::NeutralOut neutral{};

        // SetPosition blocks until the motor ACKs or this times out; keep it well under the enable timeout
        const units::time::second_t seed_timeout{0.02};
        const auto seed_retry_period = milliseconds(500);

        // A disagreement must persist this long before faulting
        const auto disagreement_persist_time = milliseconds(50);

        // Seeding can use any recent reading, but the agreement check needs a fresh one: comparing a
        // moving motor against a frozen reading (e.g. a short CAN dropout) would look like a disagreement
        const auto seed_max_sensor_age = milliseconds(500);
        const auto agreement_max_sensor_age = milliseconds(40);

        bool is_motor_seeded = false;
        bool is_motor_faulted = false;
        auto last_seed_attempt = steady_clock::time_point{};
        auto last_unseeded_warn = steady_clock::time_point{};
        auto last_overrun_warn = steady_clock::time_point{};
        int overruns_since_warn = 0;
        std::optional<steady_clock::time_point> disagreement_start;

        while (_running)
        {
            next_tick += period;

            // Motor disables itself if this stops being fed (e.g. drivebrain hangs)
            ctre::phoenix::unmanaged::FeedEnable(_config.enable_timeout_ms);

            // The motor's encoder resets to 0 if it reboots (e.g. a brownout), though its config persists.
            // Re-seed instead of letting the agreement check fault on the bogus position.
            if (is_motor_seeded && !is_motor_faulted && _kraken->HasResetOccurred())
            {
                spdlog::warn("KrakenComms: motor reset detected, re-seeding position from steering sensor");
                is_motor_seeded = false;
                disagreement_start.reset();
            }

            auto [steering_deg, steering_valid] = core::StateTracker::instance().get_steering_angle_and_validity(
                is_motor_seeded ? agreement_max_sensor_age : seed_max_sensor_age);

            if (is_motor_faulted)
            {
                _kraken->SetControl(neutral);
            }
            else if (!is_motor_seeded)
            {
                bool is_seed_ok = false;
                if (steering_valid && steady_clock::now() - last_seed_attempt >= seed_retry_period)
                {
                    last_seed_attempt = steady_clock::now();
                    is_seed_ok = _kraken->SetPosition(_deg_to_turns(steering_deg), seed_timeout).IsOK();
                }

                if (is_seed_ok)
                {
                    {
                        std::scoped_lock lock(_angle_mutex);
                        _angle_deg = std::clamp(steering_deg, _config.min_angle_deg, _config.max_angle_deg);
                    }
                    is_motor_seeded = true;
                    spdlog::info("KrakenComms: seeded motor position to {} deg from steering sensor", steering_deg);
                }
                else
                {
                    _kraken->SetControl(neutral);
                    if (steady_clock::now() - last_unseeded_warn > seconds(1))
                    {
                        spdlog::warn("KrakenComms: waiting to seed motor position (steering sensor valid: {})", steering_valid);
                        last_unseeded_warn = steady_clock::now();
                    }
                }
            }
            else // no fault and seeded branch
            {
                if (steering_valid)
                {
                    const float measured_deg = get_measured_angle_deg();
                    const float disagreement_deg = std::abs(measured_deg - steering_deg);
                    if (disagreement_deg > _config.max_disagreement_deg)
                    {
                        if (!disagreement_start)
                        {
                            disagreement_start = steady_clock::now();
                        }
                        else if (steady_clock::now() - *disagreement_start >= disagreement_persist_time)
                        {
                            is_motor_faulted = true;
                            _kraken->SetControl(neutral);
                            spdlog::error("KrakenComms: FAULT motor angle {} deg disagrees with steering sensor {} deg "
                                          "(limit {} deg). Motor set to neutral until restart. Check Inverted and sensor sign.",
                                          measured_deg, steering_deg, _config.max_disagreement_deg);
                        }
                    }
                    else
                    {
                        disagreement_start.reset();
                    }
                }

                if (!is_motor_faulted)
                {
                    float target_deg;
                    if (_test_mode)
                    {
                        target_deg = std::clamp(_test_angle_deg.load(), _config.min_angle_deg, _config.max_angle_deg);
                    }
                    else
                    {
                        std::scoped_lock lock(_angle_mutex);
                        target_deg = _angle_deg;
                    }

                    _kraken->SetControl(request.WithPosition(_deg_to_turns(target_deg)));
                }
            }

            auto now = steady_clock::now();
            if (now > next_tick)
            {
                // Rate-limited so a stalled bus can't flood the log
                ++overruns_since_warn;
                if (now - last_overrun_warn > seconds(1))
                {
                    spdlog::warn("KrakenComms loop overrun by {}us ({} overruns in the last second)",
                                 duration_cast<microseconds>(now - next_tick).count(), overruns_since_warn);
                    last_overrun_warn = now;
                    overruns_since_warn = 0;
                }
                next_tick = now;
            }
            std::this_thread::sleep_until(next_tick);
        }
    }
}