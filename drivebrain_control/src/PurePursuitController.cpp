#include "PurePursuitController.hpp"

#include <Literals.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

bool control::PurePursuitController::init() {
    return true;
}

ControllerOutput control::PurePursuitController::step(const VehicleState& in) {
    const auto path = StateTracker::instance().dv_state().path;

    SpeedControlOut speed_out = {};
    speed_out.mcu_recv_millis = in.prev_MCU_recv_millis;
    speed_out.torque_lim_nm = {_torque_lim, _torque_lim, _torque_lim, _torque_lim};

    ControllerOutput out = {};
    auto steering = path ? _steering(in, *path) : std::nullopt;
    if (steering) {
        float speed = _constant_speed;
        const auto now = std::chrono::steady_clock::now();
        if (_speed_profile && _track_closed) {
            const auto profile = speed_profile(*path, *_speed_profile);
            const float target = profile[steering->closest];
            // Braking follows the profile at once (it already slows down ahead of corners); speeding up is rate-limited
            const float dt = std::min(std::chrono::duration<float>(now - _last_step_time).count(), 0.1f);
            _commanded_speed = target > _commanded_speed ? std::min(target, _commanded_speed + _speed_profile->max_accel_mps2 * dt) : target;
            speed = _commanded_speed;
        } else {
            _commanded_speed = _constant_speed;  // the profile starts from the constant speed the car was already holding
        }
        _last_step_time = now;
        if (_finished) speed = 0.0f;
        const float rpm = speed * constants::METERS_PER_SECOND_TO_RPM;
        speed_out.desired_rpms = {rpm, rpm, rpm, rpm};
        out.desired_steering_deg = steering->angle_rad * 180.0f / M_PI;
        out.lookahead_point_map_frame = steering->target;
    } else {
        speed_out.desired_rpms = {0.0f, 0.0f, 0.0f, 0.0f};
    }
    out.out = speed_out;
    return out;
}

std::optional<control::PurePursuitController::Steering> control::PurePursuitController::_steering(const VehicleState& in, const std::vector<xyz_vec<float>>& path) const {
    if (path.size() < 2) {
        return std::nullopt;
    }
    const auto& pos = in.vehicle_position_map_frame;
    const auto& heading = in.vehicle_heading_map_frame_unit_vector;
    auto dist = [](const xyz_vec<float>& a, const xyz_vec<float>& b) { return std::hypot(b.x - a.x, b.y - a.y); };

    size_t closest = 0;
    for (size_t i = 1; i < path.size(); ++i) {
        if (dist(pos, path[i]) < dist(pos, path[closest])) {
            closest = i;
        }
    }

    xyz_vec<float> target = path.back();
    float remaining = _lookahead_dist;
    for (size_t i = closest; i + 1 < path.size(); ++i) {
        const float seg = dist(path[i], path[i + 1]);
        if (seg >= remaining) {
            const float t = remaining / seg;
            target = {path[i].x + t * (path[i + 1].x - path[i].x), path[i].y + t * (path[i + 1].y - path[i].y), 0.0f};
            break;
        }
        remaining -= seg;
    }

    const float dx = target.x - pos.x;
    const float dy = target.y - pos.y;
    const float dist_sq = dx * dx + dy * dy;
    if (dist_sq < 1e-4f) {
        return std::nullopt;
    }
    const float lateral = heading.x * dy - heading.y * dx;
    return Steering{std::atan(2.0f * lateral / dist_sq * WHEELBASE_M), target, closest};
}
