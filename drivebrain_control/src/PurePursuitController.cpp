#include "PurePursuitController.hpp"

#include <FoxgloveServer.hpp>
#include <Literals.hpp>

#include <algorithm>
#include <cmath>

bool control::PurePursuitController::init() {
    auto& foxglove = FoxgloveServer::instance();
    auto lookahead = foxglove.get_param<float>("purepursuitcontroller/lookahead_dist_m");
    auto min_speed = foxglove.get_param<float>("purepursuitcontroller/min_speed_mps");
    auto max_speed = foxglove.get_param<float>("purepursuitcontroller/max_speed_mps");
    auto max_lat_accel = foxglove.get_param<float>("purepursuitcontroller/max_lat_accel_mps2");
    auto torque_lim = foxglove.get_param<float>("purepursuitcontroller/torque_lim_nm");
    if (!lookahead || !min_speed || !max_speed || !max_lat_accel || !torque_lim) {
        spdlog::error("Couldn't load all params for pure pursuit controller.");
        return false;
    }
    _lookahead_dist = *lookahead;
    _min_speed = *min_speed;
    _max_speed = *max_speed;
    _max_lat_accel = *max_lat_accel;
    _torque_lim = *torque_lim;
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
        const float speed = std::clamp(std::sqrt(_max_lat_accel / std::max(std::fabs(steering->curvature), 1e-3f)), _min_speed, _max_speed);
        const float rpm = speed * constants::METERS_PER_SECOND_TO_RPM;
        speed_out.desired_rpms = {rpm, rpm, rpm, rpm};
        out.desired_steering_deg = steering->angle_rad * 180.0f / M_PI;
        spdlog::info("steer deg: {}", out.desired_steering_deg.value());
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
    const float curvature = 2.0f * lateral / dist_sq;
    return Steering{std::atan(curvature * WHEELBASE_M), curvature, target};
}
