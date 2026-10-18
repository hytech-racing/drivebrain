#include <KrakenComms.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

#include <fmt/ranges.h>

#include <ctre/phoenix6/unmanaged/Unmanaged.hpp>
#include <google/protobuf/wrappers.pb.h>
#include <spdlog/spdlog.h>

#include "Telemetry.hpp"



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
        auto supply_current_limit_a = foxglove.get_param<float>("kraken_comms/supply_current_limit_a");
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
              stator_current_limit_a && supply_current_limit_a && enable_timeout_ms && max_disagreement_deg &&
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
            .supply_current_limit_a = supply_current_limit_a.value(),
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

        std::cout << "KRAKEN INITIALIZED!!!!" << std::endl;

        ctre::phoenix6::configs::TalonFXConfiguration talon_config{};
        talon_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        
        talon_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

        talon_config.Feedback.SensorToMechanismRatio = units::dimensionless::scalar_t{_config.reduction};

        talon_config.Slot0 = _make_slot0_configs(_tuning_params);
        talon_config.MotionMagic = _make_motion_magic_configs(_tuning_params);

        talon_config.CurrentLimits.StatorCurrentLimit = units::current::ampere_t{_config.stator_current_limit_a};
        talon_config.CurrentLimits.StatorCurrentLimitEnable = true;
        talon_config.CurrentLimits.SupplyCurrentLimit = units::current::ampere_t{_config.supply_current_limit_a};
        talon_config.CurrentLimits.SupplyCurrentLimitEnable = true;

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

        // Sticky faults survive until cleared, so these may be left over from before this run
        spdlog::info("KrakenComms: sticky faults at startup: {}", _describe_faults(true));

        _test_mode = foxglove.get_param<bool>("kraken_comms/test_mode").value_or(false);
        _use_test_angle = foxglove.get_param<bool>("kraken_comms/use_test_angle").value_or(false);
        _test_angle_deg = foxglove.get_param<float>("kraken_comms/test_angle_deg").value_or(0.0f);
        _use_sine_test = foxglove.get_param<bool>("kraken_comms/use_sine_test").value_or(false);
        _sine_speed_scale = foxglove.get_param<float>("kraken_comms/sine_speed_scale").value_or(1.0f);
        if (_test_mode)
        {
            if (_use_sine_test)
            {
                spdlog::warn("KrakenComms: test_mode enabled, targeting sine test at {}x speed", _sine_speed_scale.load());
            }
            else if (_use_test_angle)
            {
                spdlog::warn("KrakenComms: test_mode enabled, targeting test angle {} deg", _test_angle_deg.load());
            }
            else
            {
                spdlog::warn("KrakenComms: test_mode enabled, targeting commanded angle");
            }
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

        if (auto is_test_angle_on = core::process_param_update<bool>(new_param_map, "kraken_comms/use_test_angle"))
        {
            if (*is_test_angle_on != _use_test_angle.load())
            {
                spdlog::warn("KrakenComms: use_test_angle {}", *is_test_angle_on ? "enabled" : "disabled");
            }
            _use_test_angle = *is_test_angle_on;
        }

        if (auto is_sine_test_on = core::process_param_update<bool>(new_param_map, "kraken_comms/use_sine_test"))
        {
            if (*is_sine_test_on != _use_sine_test.load())
            {
                spdlog::warn("KrakenComms: use_sine_test {}", *is_sine_test_on ? "enabled" : "disabled");
            }
            _use_sine_test = *is_sine_test_on;
        }

        if (auto new_sine_speed_scale = core::process_param_update<float>(new_param_map, "kraken_comms/sine_speed_scale"))
        {
            _sine_speed_scale = *new_sine_speed_scale;
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

    std::string KrakenComms::_describe_faults(bool sticky)
    {
        std::vector<std::string> names;

#define KRAKEN_CHECK_FAULT(name)                                                                      \
        if ((sticky ? _kraken->GetStickyFault_##name() : _kraken->GetFault_##name()).GetValue()) \
        {                                                                                            \
            names.emplace_back(#name);                                                               \
        }

        KRAKEN_CHECK_FAULT(Hardware)
        KRAKEN_CHECK_FAULT(ProcTemp)
        KRAKEN_CHECK_FAULT(DeviceTemp)
        KRAKEN_CHECK_FAULT(Undervoltage)
        KRAKEN_CHECK_FAULT(BootDuringEnable)
        KRAKEN_CHECK_FAULT(UnlicensedFeatureInUse)
        KRAKEN_CHECK_FAULT(BridgeBrownout)
        KRAKEN_CHECK_FAULT(OverSupplyV)
        KRAKEN_CHECK_FAULT(UnstableSupplyV)
        KRAKEN_CHECK_FAULT(ForwardSoftLimit)
        KRAKEN_CHECK_FAULT(ReverseSoftLimit)
        KRAKEN_CHECK_FAULT(StatorCurrLimit)
        KRAKEN_CHECK_FAULT(SupplyCurrLimit)
        KRAKEN_CHECK_FAULT(StaticBrakeDisabled)

#undef KRAKEN_CHECK_FAULT

        return names.empty() ? "none" : fmt::format("{}", fmt::join(names, ", "));
    }

    ctre::phoenix6::configs::Slot0Configs KrakenComms::_make_slot0_configs(const tuningParams_s &tuning_params)
    {
        ctre::phoenix6::configs::Slot0Configs slot0{};
        slot0.kP = units::dimensionless::scalar_t{tuning_params.kraken_P};
        slot0.kI = units::dimensionless::scalar_t{tuning_params.kraken_I};
        slot0.kD = units::dimensionless::scalar_t{tuning_params.kraken_D};
        slot0.kS = units::dimensionless::scalar_t{tuning_params.kraken_S};
        slot0.kV = units::dimensionless::scalar_t{tuning_params.kraken_V};
        slot0.kA = units::dimensionless::scalar_t{tuning_params.kraken_A};        return slot0;
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

        ctre::phoenix6::controls::MotionMagicVoltage request{units::angle::turn_t{0.0}};
        request.WithEnableFOC(true);
        ctre::phoenix6::controls::NeutralOut neutral{};

        // SetPosition blocks until the motor ACKs or this times out; keep it well under the enable timeout
        const units::time::second_t seed_timeout{0.02};
        const auto seed_retry_period = milliseconds(500);

        bool is_motor_seeded = false;
        bool is_motor_faulted = false;
        auto last_seed_attempt = steady_clock::time_point{};
        auto last_unseeded_warn = steady_clock::time_point{};
        auto last_overrun_warn = steady_clock::time_point{};
        int overruns_since_warn = 0;

        const auto telem_period = milliseconds(20);
        auto last_telem = steady_clock::time_point{};
        float last_target_deg = 0.0f;

        // Fault changes are coalesced so a limit fault toggling every tick doesn't flood the log
        const auto fault_log_period = milliseconds(500);
        auto last_fault_log = steady_clock::time_point{};
        int last_logged_fault_field = 0;
        bool is_fault_signal_ok = true;

        // Sine test sweeps -80 to 80 deg, restarting from 0 deg each time it is enabled.
        // Phase is accumulated per tick so changing sine_speed_scale mid-test doesn't jump the target.
        constexpr float sine_amplitude_deg = 80.0f;
        constexpr float sine_period_s = 4.0f;
        bool was_sine_running = false;
        bool was_test_mode = false;
        float sine_phase_rad = 0.0f;
        auto last_sine_tick = steady_clock::time_point{};

        while (_running)
        {
            next_tick += period;

            // Motor disables itself if this stops being fed
            ctre::phoenix::unmanaged::FeedEnable(_config.enable_timeout_ms);


            if (is_motor_seeded && !is_motor_faulted && _kraken->HasResetOccurred())
            {
                is_motor_faulted = true;
                spdlog::error("KrakenComms: FAULT motor reset detected, position reference lost. "
                              "Motor set to neutral until restart.");
            }

            if (!is_motor_seeded)
            {
                if (steady_clock::now() - last_seed_attempt >= seed_retry_period)
                {
                    last_seed_attempt = steady_clock::now();
                    const auto state_and_validity = core::StateTracker::instance().get_latest_state_and_validity();
                    const auto bootup_steer_deg = state_and_validity.first.steering_column_angle_deg;
                    if (state_and_validity.first.updated_steering_estimator)
                    {
                        is_motor_seeded = _kraken->SetPosition(_deg_to_turns(bootup_steer_deg), seed_timeout).IsOK();
                    }
                }
            }

            if (is_motor_faulted)
            {
                _kraken->SetControl(neutral);
            }
            else if (!is_motor_seeded)
            {
                _kraken->SetControl(neutral);
            }
            else 
            {
                const bool is_sine_running = _test_mode && _use_sine_test;
                const auto now = steady_clock::now();
                if (is_sine_running && !was_sine_running)
                {
                    sine_phase_rad = 0.0f;
                }
                else if (is_sine_running)
                {
                    const float dt_s = duration<float>(now - last_sine_tick).count();
                    sine_phase_rad += 2.0f * static_cast<float>(M_PI) * _sine_speed_scale.load() * dt_s / sine_period_s;
                    sine_phase_rad = std::fmod(sine_phase_rad, 2.0f * static_cast<float>(M_PI));
                }
                last_sine_tick = now;
                was_sine_running = is_sine_running;

                const bool is_test_mode = _test_mode;
                if (is_test_mode && !was_test_mode)
                {
                    // Hold where the motor is until something else commands an angle
                    set_angle(get_measured_angle_deg());
                }
                was_test_mode = is_test_mode;

                if (is_test_mode)
                {
                    float input_deg = _use_test_angle ? _test_angle_deg.load() : get_commanded_angle_deg();
                    if (is_sine_running)
                    {
                        input_deg = sine_amplitude_deg * std::sin(sine_phase_rad);
                    }
                    const float target_deg = std::clamp(input_deg, _config.min_angle_deg, _config.max_angle_deg);
                    _kraken->SetControl(request.WithPosition(_deg_to_turns(target_deg)));
                    last_target_deg = target_deg;
                }
                else
                {
                    _kraken->SetControl(neutral);
                }
            }

            const float measured_deg = get_measured_angle_deg();

            if (steady_clock::now() - last_telem >= telem_period)
            {
                auto &fault_signal = _kraken->GetFaultField();
                if (!fault_signal.GetStatus().IsOK())
                {
                    if (is_fault_signal_ok)
                    {
                        spdlog::error("KrakenComms: FAULT can't read motor faults ({}), motor may be off the bus",
                                      fault_signal.GetStatus().GetName());
                    }
                    is_fault_signal_ok = false;
                }
                else
                {
                    if (!is_fault_signal_ok)
                    {
                        spdlog::info("KrakenComms: motor fault signal recovered");
                    }
                    is_fault_signal_ok = true;

                    const int fault_field = fault_signal.GetValue();
                    if (fault_field != last_logged_fault_field &&
                        steady_clock::now() - last_fault_log >= fault_log_period)
                    {
                        if (fault_field == 0)
                        {
                            spdlog::info("KrakenComms: motor faults cleared");
                        }
                        else
                        {
                            spdlog::warn("KrakenComms: motor faults active: {}", _describe_faults(false));
                        }
                        last_logged_fault_field = fault_field;
                        last_fault_log = steady_clock::now();
                    }
                }

                std::shared_ptr<hytech_msgs::KrakenInfo> msg_out = std::make_shared<hytech_msgs::KrakenInfo>();

                msg_out->set_pos_rotation(static_cast<float>(_kraken->GetPosition().GetValueAsDouble()));
                msg_out->set_steering_angle_degs(measured_deg);
                msg_out->set_velocity_deg_per_s(static_cast<float>(_kraken->GetVelocity().GetValueAsDouble() * 360.0));
                msg_out->set_setpoint_angle_degs(last_target_deg);
                msg_out->set_stator_current(static_cast<float>(_kraken->GetStatorCurrent().GetValueAsDouble()));
                msg_out->set_supply_current(static_cast<float>(_kraken->GetSupplyCurrent().GetValueAsDouble()));
                msg_out->set_temp_c(static_cast<float>(_kraken->GetDeviceTemp().GetValueAsDouble()));
                msg_out->set_applied_voltage(_kraken->GetMotorVoltage().GetValue().value());
                msg_out->set_supply_voltage(static_cast<float>(_kraken->GetSupplyVoltage().GetValueAsDouble()));
                msg_out->set_active_faults(fault_signal.GetValue() == 0 ? "none" : _describe_faults(false));

                last_telem = steady_clock::now();
                core::log(msg_out);
            }
            
            std::this_thread::sleep_until(next_tick);
        }
    }
}