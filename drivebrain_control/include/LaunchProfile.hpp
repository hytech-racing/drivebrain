#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <optional>

namespace control {

// Independent of I/O so the same profile can be exercised with a simulated clock.
class LaunchProfile {
public:
    struct Config {
        float acceleration_mps2 = 0.5f;
        float accel_distance_m = 5.0f;
        float braking_distance_m = 5.0f;
        float max_speed_mps = 2.0f;
        float max_torque_nm = 2.0f;
        float max_regen_torque_nm = 1.0f;
    };
    enum class Phase { IDLE, ACCELERATING, BRAKING, COMPLETE, FAULT };
    struct Output { float speed_mps; float torque_limit_nm; };
    struct GroundMotion { float speed_mps; double distance_m; };

    static bool valid(const Config& c) {
        auto positive = [](float v) { return std::isfinite(v) && v > 0.0f; };
        return positive(c.acceleration_mps2) && positive(c.accel_distance_m) &&
            positive(c.braking_distance_m) && positive(c.max_speed_mps) &&
            positive(c.max_torque_nm) && positive(c.max_regen_torque_nm) &&
            c.max_torque_nm <= 21.0f && c.max_regen_torque_nm <= 10.0f;
    }

    void start(const Config& config, bool healthy, const std::array<float, 4>& speeds,
               std::optional<GroundMotion> ground = std::nullopt) {
        if (_phase != Phase::IDLE && _phase != Phase::COMPLETE && _phase != Phase::FAULT) return;
        *this = LaunchProfile{};
        _config = config;
        // Starting requires every wheel stopped, not just a stationary average.
        if (!valid(config) || !healthy || (ground && (!valid_ground(*ground) || ground->speed_mps > stopped_speed_mps)) ||
            !std::all_of(speeds.begin(), speeds.end(),
            [](float v) { return std::isfinite(v) && std::abs(v) <= stopped_speed_mps; })) {
            _phase = Phase::FAULT;
            return;
        }
        _phase = Phase::ACCELERATING;
        _previous_speed = mean_speed(speeds);
        if (ground) _start_odometer_m = ground->distance_m;
    }

    Output step(float dt, const std::array<float, 4>& speeds, bool healthy, bool brake,
                std::optional<GroundMotion> ground = std::nullopt) {
        if (_phase == Phase::IDLE || _phase == Phase::FAULT) return {};
        if (!healthy || (ground && !valid_ground(*ground)) || !std::isfinite(dt) || dt < 0.0f || dt > max_step_sec ||
            !std::all_of(speeds.begin(), speeds.end(), [](float v) { return std::isfinite(v); })) {
            _phase = Phase::FAULT;
            return {};
        }
        if (_phase == Phase::COMPLETE) return {0.0f, _config.max_regen_torque_nm};
        const float speed = ground ? ground->speed_mps : mean_speed(speeds);
        if (ground) _distance = static_cast<float>(std::max(0.0, ground->distance_m - _start_odometer_m));
        else _distance += 0.5f * (_previous_speed + speed) * dt;
        _previous_speed = speed;
        if (_phase == Phase::ACCELERATING && (brake || _distance >= _config.accel_distance_m)) {
            _phase = Phase::BRAKING;
            _brake_start_distance = _distance;
            _target = speed;
            _deceleration = speed * speed / (2.0f * _config.braking_distance_m);
            // Start the brake ramp from the measured entry speed on the next tick.
            dt = 0.0f;
        }
        if (_phase == Phase::ACCELERATING) {
            _target = std::min(_config.max_speed_mps, _target + _config.acceleration_mps2 * dt);
            return {_target, _config.max_torque_nm};
        }
        _target = std::max(0.0f, _target - _deceleration * dt);
        if (_distance - _brake_start_distance >= _config.braking_distance_m) _target = 0.0f;
        if (_target <= stopped_speed_mps && speed <= stopped_speed_mps && std::all_of(speeds.begin(), speeds.end(),
            [](float v) { return std::abs(v) <= stopped_speed_mps; })) {
            _phase = Phase::COMPLETE;
            _target = 0.0f;
        }
        return {_target, _config.max_regen_torque_nm};
    }

    Phase phase() const { return _phase; }
    float distance_m() const { return _distance; }
    float deceleration_mps2() const { return _deceleration; }
    static constexpr float stopped_speed_mps = 0.05f;
    static constexpr float max_step_sec = 0.1f;

private:
    static bool valid_ground(const GroundMotion& ground) {
        return std::isfinite(ground.speed_mps) && ground.speed_mps >= 0 && std::isfinite(ground.distance_m);
    }
    static float mean_speed(const std::array<float, 4>& speeds) {
        float sum = 0.0f;
        for (float speed : speeds) sum += std::abs(speed);
        return sum / 4.0f;
    }
    Config _config{};
    Phase _phase = Phase::IDLE;
    float _target = 0.0f;
    float _distance = 0.0f;
    float _brake_start_distance = 0.0f;
    float _previous_speed = 0.0f;
    float _deceleration = 0.0f;
    double _start_odometer_m = 0.0;
};
}
