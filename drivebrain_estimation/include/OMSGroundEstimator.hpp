#pragma once

#include <cmath>

namespace estimation {
// The OMS angle channel is sideslip. Heading is relative yaw, integrated using
// the sensor's sample clock (not TCP arrival time or the control-loop period).
class OMSGroundEstimator {
public:
    struct Config {
        float raw_velocity_to_mps = 1.0f / 3.6f;
        float mounting_yaw_deg = 0.0f;
        float yaw_rate_sign = 1.0f;
        float yaw_rate_bias_deg_s = 0.0f;
    };
    struct State {
        bool received = false;
        float sample_time_s = 0.0f;
        float vx_mps = 0.0f;
        float vy_mps = 0.0f;
        float speed_mps = 0.0f;
        float sideslip_deg = 0.0f;
        float yaw_rate_deg_s = 0.0f;
        double heading_deg = 0.0;
        double distance_m = 0.0;
    };

    explicit OMSGroundEstimator(Config config) : _config(config) {}
    bool update(float sample_time_s, float raw_vx, float raw_vy, float yaw_rate_deg_s) {
        if (!std::isfinite(sample_time_s) || !std::isfinite(raw_vx) || !std::isfinite(raw_vy) ||
            !std::isfinite(yaw_rate_deg_s)) return false;
        const float angle = _config.mounting_yaw_deg * pi / 180.0f;
        const float vx = _config.raw_velocity_to_mps * (std::cos(angle) * raw_vx - std::sin(angle) * raw_vy);
        const float vy = _config.raw_velocity_to_mps * (std::sin(angle) * raw_vx + std::cos(angle) * raw_vy);
        const float speed = std::hypot(vx, vy);
        const float rate = _config.yaw_rate_sign * (yaw_rate_deg_s - _config.yaw_rate_bias_deg_s);
        const float dt = sample_time_s - _state.sample_time_s;
        if (_state.received && dt == 0.0f) return false; // duplicate sample
        if (_state.received && dt > 0.0f && dt <= 0.1f) {
            _state.heading_deg += 0.5 * (_state.yaw_rate_deg_s + rate) * dt;
            _state.distance_m += 0.5 * (_state.speed_mps + speed) * dt;
        }
        // A sensor restart/disconnection establishes a new sample-time baseline;
        // do not invent motion across the missing interval.
        _state.received = true;
        _state.sample_time_s = sample_time_s;
        _state.vx_mps = vx;
        _state.vy_mps = vy;
        _state.speed_mps = speed;
        _state.sideslip_deg = speed > 0.001f ? std::atan2(vy, vx) * 180.0f / pi : 0.0f;
        _state.yaw_rate_deg_s = rate;
        return true;
    }
    const State& state() const { return _state; }

private:
    static constexpr float pi = 3.14159265358979323846f;
    Config _config;
    State _state{};
};
}
