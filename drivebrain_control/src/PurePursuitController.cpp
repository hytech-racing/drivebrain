#include "PurePursuitController.hpp"

#include <Literals.hpp>

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
    auto steering = path ? _steering_angle_rad(in, *path) : std::nullopt;
    if (steering) {
        const float rpm = _target_speed * constants::METERS_PER_SECOND_TO_RPM;
        speed_out.desired_rpms = {rpm, rpm, rpm, rpm};
        out.desired_steering_deg = *steering * 180.0f / M_PI;
    } else {
        speed_out.desired_rpms = {0.0f, 0.0f, 0.0f, 0.0f};
    }
    out.out = speed_out;
    return out;
}

std::optional<float> control::PurePursuitController::_steering_angle_rad(const VehicleState& in, const std::vector<xyz_vec<float>>& path) const {
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
    return std::atan(2.0f * lateral / dist_sq * WHEELBASE_M);
}
