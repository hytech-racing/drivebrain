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
        auto steering_timeout_ms = foxglove.get_param<int>("kraken_comms/steering_timeout_ms");

        auto kraken_P = foxglove.get_param<float>("kraken_comms/kp");
        auto kraken_I = foxglove.get_param<float>("kraken_comms/ki");
        auto kraken_D = foxglove.get_param<float>("kraken_comms/kd");
        auto kraken_S = foxglove.get_param<float>("kraken_comms/ks");
        auto kraken_S_deadband_deg = foxglove.get_param<float>("kraken_comms/ks_deadband_deg");
        auto kraken_V = foxglove.get_param<float>("kraken_comms/kv");
        auto kraken_A = foxglove.get_param<float>("kraken_comms/ka");
        auto motion_magic_cruise_velocity_deg_s = foxglove.get_param<float>("kraken_comms/mm_cruise_velocity_deg_s");
        auto motion_magic_acceleration_deg_s2 = foxglove.get_param<float>("kraken_comms/mm_acceleration_deg_s2");

        if (!(device_id && canbus_name && send_rate_hz && reduction && min_angle_deg && max_angle_deg &&
              stator_current_limit_a && supply_current_limit_a && enable_timeout_ms &&
              steering_timeout_ms && kraken_P && kraken_I && kraken_D && kraken_S && kraken_S_deadband_deg &&
              kraken_V && kraken_A && motion_magic_cruise_velocity_deg_s && motion_magic_acceleration_deg_s2))
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
            .steering_timeout_ms = steering_timeout_ms.value(),
        };

        _tuning_params = {
            .kraken_P = kraken_P.value(),
            .kraken_I = kraken_I.value(),
            .kraken_D = kraken_D.value(),
            .kraken_S = kraken_S.value(),
            .kraken_S_deadband_deg = kraken_S_deadband_deg.value(),
            .kraken_V = kraken_V.value(),
            .kraken_A = kraken_A.value(),
            .motion_magic_cruise_velocity_deg_s = motion_magic_cruise_velocity_deg_s.value(),
            .motion_magic_acceleration_deg_s2 = motion_magic_acceleration_deg_s2.value(),
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

        if (_config.steering_timeout_ms <= 0)
        {
            spdlog::error("KrakenComms: steering_timeout_ms must be > 0, got {}", _config.steering_timeout_ms);
            return false;
        }

        if (!_is_tuning_valid(_tuning_params))
        {
            return false;
        }

        _kraken = std::make_unique<ctre::phoenix6::hardware::TalonFX>(_config.device_id, _config.canbus_name);

        std::cout << "KRAKEN INITIALIZED!!!!" << std::endl;

        ctre::phoenix6::configs::TalonFXConfiguration talon_config{};
        talon_config.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;
        
        talon_config.MotorOutput.Inverted = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

        talon_config.Feedback.SensorToMechanismRatio = units::dimensionless::scalar_t{_config.reduction};

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

        // The PID reads velocity every tick, so don't let it lag behind the default status rate
        status = ctre::phoenix6::BaseStatusSignal::SetUpdateFrequencyForAll(
            units::frequency::hertz_t{static_cast<double>(_config.send_rate_hz)},
            _kraken->GetPosition(), _kraken->GetVelocity());
        if (!status.IsOK())
        {
            spdlog::warn("KrakenComms: failed to set position/velocity update rate: {}", status.GetName());
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
        update("kraken_comms/ks_deadband_deg", new_tuning.kraken_S_deadband_deg);
        update("kraken_comms/kv", new_tuning.kraken_V);
        update("kraken_comms/ka", new_tuning.kraken_A);
        update("kraken_comms/mm_cruise_velocity_deg_s", new_tuning.motion_magic_cruise_velocity_deg_s);
        update("kraken_comms/mm_acceleration_deg_s2", new_tuning.motion_magic_acceleration_deg_s2);

        std::scoped_lock lock(_tuning_mutex);
        if (!(new_tuning == _tuning_params) && _is_tuning_valid(new_tuning))
        {
            _tuning_params = new_tuning;
            spdlog::info("KrakenComms: applied tuning kP={} kI={} kD={} kS={} kS_deadband={}deg kV={} kA={} "
                         "cruise={}deg/s accel={}deg/s^2",
                         new_tuning.kraken_P, new_tuning.kraken_I, new_tuning.kraken_D, new_tuning.kraken_S,
                         new_tuning.kraken_S_deadband_deg, new_tuning.kraken_V, new_tuning.kraken_A,
                         new_tuning.motion_magic_cruise_velocity_deg_s, new_tuning.motion_magic_acceleration_deg_s2);
        }
    }

    bool KrakenComms::_is_tuning_valid(const tuningParams_s &tuning_params)
    {
        if (tuning_params.motion_magic_cruise_velocity_deg_s <= 0.0f ||
            tuning_params.motion_magic_acceleration_deg_s2 <= 0.0f ||
            tuning_params.kraken_S_deadband_deg < 0.0f)
        {
            spdlog::error("KrakenComms: rejected tuning cruise={}deg/s accel={}deg/s^2 kS_deadband={}deg "
                          "(cruise and accel must be > 0, kS_deadband >= 0)",
                          tuning_params.motion_magic_cruise_velocity_deg_s,
                          tuning_params.motion_magic_acceleration_deg_s2, tuning_params.kraken_S_deadband_deg);
            return false;
        }
        return true;
    }

    KrakenComms::TrapezoidalProfile::state_s KrakenComms::TrapezoidalProfile::step(float goal_deg, float max_vel_deg_s,
                                                                                   float max_acc_deg_s2, float dt_s)
    {
        // Solve the whole remaining trapezoid from the current state each tick, then sample it dt ahead.
        // Re-solving keeps it exact when the goal moves mid-move and lands precisely on the goal at the end.

        // Pick the direction to travel: toward the goal from wherever we'd stop if we braked now
        const float stop_offset_deg = _state.vel_deg_s * std::abs(_state.vel_deg_s) / (2.0f * max_acc_deg_s2);
        const float dir = (goal_deg - _state.pos_deg - stop_offset_deg) >= 0.0f ? 1.0f : -1.0f;

        // Work in a flipped frame where travel is always positive
        const float dist_deg = dir * (goal_deg - _state.pos_deg);
        const float v0 = dir * _state.vel_deg_s;
        const float a = max_acc_deg_s2;

        // Peak speed: cruise if there's room, otherwise the triangle peak where accel and decel meet
        const float peak_vel = std::min(max_vel_deg_s, std::sqrt(std::max(0.0f, (2.0f * a * dist_deg + v0 * v0) / 2.0f)));

        // Phase 1 ramps v0 to peak (may be a decel if v0 is above cruise or a speed up through zero if we were
        // moving away), phase 2 cruises, phase 3 brakes to zero on the goal
        const float a1 = peak_vel >= v0 ? a : -a;
        const float t1 = std::abs(peak_vel - v0) / a;
        const float d1 = (peak_vel * peak_vel - v0 * v0) / (2.0f * a1);
        const float t3 = peak_vel / a;
        const float d3 = peak_vel * peak_vel / (2.0f * a);
        const float d2 = std::max(0.0f, dist_deg - d1 - d3);
        const float t2 = peak_vel > 1e-6f ? d2 / peak_vel : 0.0f;

        float pos, vel, acc;
        if (dt_s >= t1 + t2 + t3)
        {
            _state = {.pos_deg = goal_deg, .vel_deg_s = 0.0f, .acc_deg_s2 = 0.0f};
            return _state;
        }
        else if (dt_s <= t1)
        {
            pos = v0 * dt_s + 0.5f * a1 * dt_s * dt_s;
            vel = v0 + a1 * dt_s;
            acc = a1;
        }
        else if (dt_s <= t1 + t2)
        {
            pos = d1 + peak_vel * (dt_s - t1);
            vel = peak_vel;
            acc = 0.0f;
        }
        else
        {
            const float tau = dt_s - t1 - t2;
            pos = d1 + d2 + peak_vel * tau - 0.5f * a * tau * tau;
            vel = peak_vel - a * tau;
            acc = -a;
        }

        _state = {.pos_deg = _state.pos_deg + dir * pos, .vel_deg_s = dir * vel, .acc_deg_s2 = dir * acc};
        return _state;
    }

    void KrakenComms::TrapezoidalProfile::reset(float pos_deg, float vel_deg_s)
    {
        _state = {.pos_deg = pos_deg, .vel_deg_s = vel_deg_s, .acc_deg_s2 = 0.0f};
    }

    float KrakenComms::ColumnAnglePID::step(const tuningParams_s &tuning, const TrapezoidalProfile::state_s &reference,
                                            float measured_deg, float measured_velocity_deg_s, float dt_s)
    {
        // Gains are in Slot0 units, so do the math in mechanism rotations
        const float error_deg = reference.pos_deg - measured_deg;
        const float error_rot = error_deg / 360.0f;
        const float ref_vel_rps = reference.vel_deg_s / 360.0f;
        const float ref_acc_rps2 = reference.acc_deg_s2 / 360.0f;

        _integral_v += tuning.kraken_I * error_rot * dt_s;

        const float p_v = tuning.kraken_P * error_rot;
        // The profile velocity is continuous, so D on velocity error doesn't kick when the target steps
        const float d_v = tuning.kraken_D * (ref_vel_rps - measured_velocity_deg_s / 360.0f);

        float s_v = 0.0f;
        if (reference.vel_deg_s != 0.0f)
        {
            s_v = std::copysign(tuning.kraken_S, reference.vel_deg_s);
        }
        else if (std::abs(error_deg) > tuning.kraken_S_deadband_deg)
        {
            s_v = std::copysign(tuning.kraken_S, error_deg);
        }
        const float ff_v = s_v + tuning.kraken_V * ref_vel_rps + tuning.kraken_A * ref_acc_rps2;

        return p_v + _integral_v + d_v + ff_v;
    }

    void KrakenComms::ColumnAnglePID::reset()
    {
        _integral_v = 0.0f;
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

    units::angle::turn_t KrakenComms::_deg_to_turns(double degree)
    {
        return units::angle::turn_t{degree / 360.0};
    }

    void KrakenComms::_run()
    {
        using namespace std::chrono;

        const auto period = microseconds(static_cast<long long>(1'000'000.0 / _config.send_rate_hz));
        auto next_tick = steady_clock::now();

        ctre::phoenix6::controls::VoltageOut request{units::voltage::volt_t{0.0}};
        request.WithEnableFOC(true);

        TrapezoidalProfile profile{};
        ColumnAnglePID pid{};
        bool was_pid_active = false;
        auto last_pid_tick = steady_clock::time_point{};
        const auto steering_timeout = milliseconds(_config.steering_timeout_ms);
        bool was_steering_fresh = false;
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

            const auto vehicle_state = core::StateTracker::instance().get_latest_state_and_validity().first;
            const float column_deg = vehicle_state.steering_column_angle_deg;
            const bool is_steering_fresh = vehicle_state.updated_steering_estimator &&
                                           steady_clock::now() - vehicle_state.last_steering_recv_time < steering_timeout;
            if (is_steering_fresh != was_steering_fresh)
            {
                if (is_steering_fresh)
                {
                    spdlog::info("KrakenComms: steering column angle feedback recovered");
                }
                else
                {
                    spdlog::error("KrakenComms: FAULT steering column angle older than {} ms, motor set to neutral",
                                  _config.steering_timeout_ms);
                }
            }
            was_steering_fresh = is_steering_fresh;

            if (!is_motor_seeded && is_steering_fresh &&
                steady_clock::now() - last_seed_attempt >= seed_retry_period)
            {
                last_seed_attempt = steady_clock::now();
                is_motor_seeded = _kraken->SetPosition(_deg_to_turns(column_deg), seed_timeout).IsOK();
            }

            bool is_pid_active = false;

            if (is_motor_faulted || !is_motor_seeded || !is_steering_fresh)
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
                    // Hold where the column is until something else commands an angle
                    set_angle(column_deg);
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

                    const float column_velocity_deg_s = static_cast<float>(_kraken->GetVelocity().GetValueAsDouble() * 360.0);

                    if (!was_pid_active)
                    {
                        // Start the profile from where the column is so engaging doesn't jump the reference
                        profile.reset(column_deg, column_velocity_deg_s);
                        pid.reset();
                    }
                    const float dt_s = was_pid_active ? duration<float>(now - last_pid_tick).count()
                                                      : duration<float>(period).count();
                    last_pid_tick = now;

                    tuningParams_s tuning{};
                    {
                        std::scoped_lock lock(_tuning_mutex);
                        tuning = _tuning_params;
                    }
                    const auto reference = profile.step(target_deg, tuning.motion_magic_cruise_velocity_deg_s,
                                                        tuning.motion_magic_acceleration_deg_s2, dt_s);
                    const float output_v = pid.step(tuning, reference, column_deg, column_velocity_deg_s, dt_s);

                    _kraken->SetControl(request.WithOutput(units::voltage::volt_t{output_v}));
                    last_target_deg = target_deg;
                    is_pid_active = true;
                }
                else
                {
                    _kraken->SetControl(neutral);
                }
            }

            was_pid_active = is_pid_active;

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