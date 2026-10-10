#include "VelocityPlanner.hpp"
#include "Literals.hpp"
#include <Mathematics/NaturalCubicSpline.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <optional>
#include <spdlog/spdlog.h>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace planning {

constexpr float kGravity = 9.80665f;
constexpr float kMinPathTangent = 1e-6f;
const float kCarWeightKg = 180.0f;
const float kMu = 1;
const float kMaxAccel = kMu * kGravity * 1.01f;
const float pathStaleLookaheadRatio = 0.3f; // if we haven't gotten an update from path planner and we passed by 0.3 * lookahead distance points, flag it

VelocityPlanner::VelocityPlanner(float max_car_velocity)
    : max_car_velocity_(max_car_velocity) {
        path_points_.reserve(lookahead_distance_);
}

std::size_t VelocityPlanner::getHorizonPointIndex(const core::VehicleState& state) {
    const core::xy_vec<float> position{state.vehicle_position_map_frame.x, state.vehicle_position_map_frame.y}; // global frame
    const core::xy_vec<float> heading{state.vehicle_heading_map_frame_unit_vector.x, state.vehicle_heading_map_frame_unit_vector.y}; 


    // iterate with offset to allow for wraparound
    size_t index = 0;
    while (index < path_.size()) {
        const core::xy_vec<float> to_point = path_[index] - position;
        if (heading * to_point < 0.0f) { // point is behind the car, skip it
            ++index;
            continue;
        }
        return index;
    }
    spdlog::error("Closest point not found");
    return std::numeric_limits<size_t>::max();
}

core::TorqueControlOut VelocityPlanner::step_controller(const core::VehicleState& state, const float pose_to_path_curvature) {
    core::TorqueControlOut out = {};
    if (path_.empty()) {
        spdlog::error("Calling velocity controller on an empty path");
        return out;
    }

    if (path_.size() < lookahead_distance_) {
        spdlog::error("Path is too short, only {} points, need at least {}", path_.size(), lookahead_distance_);
        return out;
    }

    start_point_index_= getHorizonPointIndex(state);
    if (start_point_index_ == std::numeric_limits<size_t>::max()) {
        spdlog::error("Failed to find closest point on path");
        return out;
    }

    if (start_point_index_ >= static_cast<size_t>(lookahead_distance_ * pathStaleLookaheadRatio)) {
        spdlog::error("Path is stale, we have passed by {} lookahead points", start_point_index_);
        return out;
    }

    path_points_.resize(std::min(lookahead_distance_, path_.size() - start_point_index_));
    std::optional<float> accel = getAccel(state, pose_to_path_curvature, start_point_index_);
    if (!accel.has_value()) {
        spdlog::error("Failed to compute acceleration");
        return out;
    }

    const float torque = accel.value() * kCarWeightKg * (constants::WHEEL_DIAMETER / 2.0f) / (4.0f * constants::GEARBOX_RATIO);
    out.desired_torques_nm.FL = torque;
    out.desired_torques_nm.FR = torque;
    out.desired_torques_nm.RL = torque;
    out.desired_torques_nm.RR = torque;
    return out;
}

const std::optional<float> VelocityPlanner::getAccel(const core::VehicleState& state, const float pose_to_path_curvature, size_t start_point_index) {
    size_t end_point_index = std::min(start_point_index + lookahead_distance_ - 1, path_.size() - 1);
    const size_t path_length = end_point_index - start_point_index + 1;

    // add start point to cumul distance parametrization parametrization 
    std::vector<float> times;
    times.reserve(path_length);
    times.push_back(0.0f);

    // copy points from core_xy_vec to gte::Vector<2, float>
    std::vector<gte::Vector<2, float>> positions;
    positions.reserve(path_length);
    positions.push_back({path_[start_point_index].x, path_[start_point_index].y});

    // parametrize the path by cumulative distance along the path (chord length)
    for (std::size_t i = 1; i < path_length; ++i) {
        positions.push_back({path_[i + start_point_index].x, path_[i + start_point_index].y});
        const float d = (path_[i + start_point_index] - path_[i + start_point_index - 1]).length();
        times.push_back(times.back() + d);
        if (times.back() <= times[i - 1]) {
            spdlog::error("Path points not in order or too close");
            return {};
        }
    }

    // fit the spline
    gte::NaturalCubicSpline<2, float> spline(true, positions, times);


    std::vector<PathPoint> samples;


    for (size_t i = 0; i < path_length; ++i) {
        // for given spline section, get r(t), r't(t), r''(t) (IMPORTANT T IS NOT TIME)
        std::array<gte::Vector<2, float>, 3> jet{};
        // subtract index since path used in times starts at the point in front of the car (start index)
        spline.Evaluate(times[i], 2, jet.data()); // evaluate at cumul distance times[t]
        gte::Vector<2, float> position = jet[0];
        gte::Vector<2, float> path_tangent = jet[1]; // with respect to spline parameter t (cumulative distance along path)
        gte::Vector<2, float> change_in_tangent = jet[2];
            
        const float path_tangent_squared = path_tangent[0] * path_tangent[0] + path_tangent[1] * path_tangent[1];

        if (path_tangent_squared <= kMinPathTangent) {
            spdlog::error("Spline tangent is too small, cannot compute curvature");
            return {};
        }

        const float cross = path_tangent[0] * change_in_tangent[1] - path_tangent[1] * change_in_tangent[0];
        const float curvature = cross / (path_tangent_squared * std::sqrt(path_tangent_squared));

        // sqrt(ax+ay) = mu * g where ax == 0 (ay = v*v*k) => v = sqrt(mu * g / k)
        const float v_max = std::min(max_car_velocity_, std::sqrt(kMu * kGravity / std::abs(curvature)));
        path_points_[i] = {
            .point = {position[0], position[1]},
            .curvature = curvature,
            .velocity = v_max,
        };
    }
    
    // run forward and backwards solver
    if (!solver(state, pose_to_path_curvature, path_length, true)) return {}; 
    if (!solver(state, pose_to_path_curvature, path_length, false)) return {};

    // calculate accel from target speed at closest path point and current point (a = (v^2 - u^2) / 2d)
    const core::xy_vec<float> position{state.vehicle_position_map_frame.x, state.vehicle_position_map_frame.y}; // global frame
    const core::xy_vec<float> velocity{state.current_body_vel_ms.x, state.current_body_vel_ms.y}; // local frame
    const core::xy_vec<float> dist = position - path_points_[0].point;
    if (dist.length() < 1e-6f) {
        spdlog::error("Distance to closest path point is too small, cannot compute acceleration");
        return {};
    }
    const float accel = (path_points_[0].velocity * path_points_[0].velocity - velocity.length() * velocity.length()) / (2 * dist.length()); 
    return std::make_optional(accel);
}

bool VelocityPlanner::solver(const core::VehicleState& state, const float pose_to_path_curvature, const int path_length, bool forward) {
    const core::xy_vec<float> position{state.vehicle_position_map_frame.x, state.vehicle_position_map_frame.y}; // global frame
    const core::xy_vec<float> velocity{state.current_body_vel_ms.x, state.current_body_vel_ms.y}; // local frame
    const float speed = velocity.length();

    PathPoint current_point = {
        .point = position,
        .curvature = pose_to_path_curvature,
        .velocity = speed, 
    };

    PathPoint start_point;
    PathPoint saved_point;
    if (forward) {
        start_point = current_point; // start evaluation from current car position
    } else {
        start_point = path_points_.back(); // evaluate from the end
        saved_point = path_points_.back(); // save the last point to put it back after reverse
        path_points_.pop_back(); // speed at start point is given, avoid reassigning it
        std::reverse(path_points_.begin(), path_points_.end());
    }

    const int max_index = forward ? path_length : path_length - 1; // if backward, we ignore the last point but dont add fist point
    for (size_t i = 0; i < max_index; ++i) {
        auto& next_point = path_points_[i];
        auto cur_to_next = (next_point.point - start_point.point);
        const float ds = cur_to_next.length();
        const float lateral_accel = std::abs(start_point.curvature) * start_point.velocity * start_point.velocity; // a_y = v^2 * k
        if (lateral_accel > kMaxAccel) { // always positive no need for abs()
            spdlog::error("Lateral acceleration exceeds maximum limit, previous point might've been too far. Curvature: {}, Velocity: {}, Lateral Accel: {}, Max Accel: {}", start_point.curvature, start_point.velocity, lateral_accel, kMaxAccel);
            if (!forward) { // restore state 
                std::reverse(path_points_.begin(), path_points_.end());
                path_points_.push_back(saved_point); 
            }
            return false;
        }
        const float a_long_avail = lateral_accel > kMaxAccel ?
         -kMaxAccel : std::sqrt( kMaxAccel * kMaxAccel - lateral_accel * lateral_accel); // set accel to -MAX if we're off limit

        const float v_long_avail = std::sqrt(start_point.velocity * start_point.velocity + 2 * a_long_avail * ds);
        next_point.velocity = std::min(next_point.velocity, v_long_avail);
        start_point = next_point;
    }

    if (!forward) {
        std::reverse(path_points_.begin(), path_points_.end());
        path_points_.push_back(saved_point); 
    }
    return true;
}

} // namespace planning
