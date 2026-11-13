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
    auto max_brake = foxglove.get_param<float>("purepursuitcontroller/max_brake_mps2");
    auto torque_lim = foxglove.get_param<float>("purepursuitcontroller/torque_lim_nm");
    if (!lookahead || !min_speed || !max_speed || !max_lat_accel || !max_brake || !torque_lim) {
        spdlog::error("Couldn't load all params for pure pursuit controller.");
        return false;
    }
    _lookahead_dist = *lookahead;
    _min_speed = *min_speed;
    _max_speed = *max_speed;
    _max_lat_accel = *max_lat_accel;
    _max_brake = *max_brake;
    _torque_lim = *torque_lim;

    _param_connection = foxglove.register_param_callback([this](const std::unordered_map<std::string, core::DBParam>& params) {
        if (auto v = core::process_param_update<float>(params, "purepursuitcontroller/lookahead_dist_m")) _lookahead_dist = *v;
        if (auto v = core::process_param_update<float>(params, "purepursuitcontroller/min_speed_mps")) _min_speed = *v;
        if (auto v = core::process_param_update<float>(params, "purepursuitcontroller/max_speed_mps")) _max_speed = *v;
        if (auto v = core::process_param_update<float>(params, "purepursuitcontroller/max_lat_accel_mps2")) _max_lat_accel = *v;
        if (auto v = core::process_param_update<float>(params, "purepursuitcontroller/max_brake_mps2")) _max_brake = *v;
        if (auto v = core::process_param_update<float>(params, "purepursuitcontroller/torque_lim_nm")) _torque_lim = *v;
    });
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
        const float rpm = _profile_speed(*path) * constants::METERS_PER_SECOND_TO_RPM;
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
    return Steering{std::atan(curvature * WHEELBASE_M), target};
}

float control::PurePursuitController::_profile_speed(const std::vector<xyz_vec<float>>& path) const {
    auto dist = [&](size_t a, size_t b) { return std::hypot(path[b].x - path[a].x, path[b].y - path[a].y); };

    float speed = _max_speed;
    float dist_along_path = 0.0f;
    for (size_t i = 1; i + 1 < path.size(); ++i) {
        const auto& prev = path[i - 1];
        const auto& curr = path[i];
        const auto& next = path[i + 1];
        const float prev_to_curr = dist(i - 1, i);
        const float curr_to_next = dist(i, i + 1);
        const float prev_to_next = dist(i - 1, i + 1);
        dist_along_path += prev_to_curr;
        if (prev_to_curr < 1e-3f || curr_to_next < 1e-3f || prev_to_next < 1e-3f) continue; // avoid divide by zero

        // How sharply the path bends at curr: 1 / radius of the circle through prev, curr and next
        const float twice_triangle_area = std::fabs((curr.x - prev.x) * (next.y - curr.y) - (curr.y - prev.y) * (next.x - curr.x));
        const float curvature = 2.0f * twice_triangle_area / (prev_to_curr * curr_to_next * prev_to_next);

        // Fastest speed through this curve without exceeding max_lat_accel (lateral accel = speed^2 * curvature)
        const float corner_speed = std::clamp(std::sqrt(_max_lat_accel / std::max(curvature, 1e-3f)), _min_speed.load(), _max_speed.load());

        // Fastest speed now that can still brake down to corner_speed by the time the car gets there
        speed = std::min(speed, std::sqrt(corner_speed * corner_speed + 2.0f * _max_brake * dist_along_path));
    }
    return speed;
}
