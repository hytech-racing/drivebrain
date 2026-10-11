#include "VelocityPlanner.hpp"
#include "Literals.hpp"
#include <Mathematics/NaturalCubicSpline.h>

#include <algorithm>
#include <array>
#include <chrono>
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
constexpr char kTracePath[] = "/tmp/velocity-planner.jsonl";
const float kCarWeightKg = 180.0f;
const float kMu = 1;
const float kMaxAccel = kMu * kGravity * 1.01f;
const float pathStaleLookaheadRatio = 0.3f; // if we haven't gotten an update from path planner and we passed by 0.3 * lookahead distance points, flag it

VelocityPlanner::VelocityPlanner(float max_car_velocity)
    : max_car_velocity_(max_car_velocity) {
        path_points_.reserve(lookahead_distance_);
        trace_file_.open(kTracePath, std::ios::out | std::ios::trunc);
        if (!trace_file_) spdlog::error("Cannot open velocity planner trace: {}", kTracePath);
}

std::size_t VelocityPlanner::getHorizonPointIndex(const core::VehicleState& state, const core::xy_vec<float>& position, const core::xy_vec<float>& heading) {
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

core::TorqueControlOut VelocityPlanner::step_controller(const core::VehicleState& state,
                                                        const float pose_to_path_curvature) {
    core::TorqueControlOut out = {};
    const core::xy_vec<float> car_position{state.vehicle_position_map_frame.x, state.vehicle_position_map_frame.y}; // global frame
    const core::xy_vec<float> car_velocity{state.current_body_vel_ms.x, state.current_body_vel_ms.y}; // TODO: currently global frame but VNData is body frame need to add transforms to drivebrain
    const core::xy_vec<float> car_heading{state.vehicle_heading_map_frame_unit_vector.x, state.vehicle_heading_map_frame_unit_vector.y}; // global frame 
    std::optional<nlohmann::json> trace;
    if (trace_file_) {
        const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        trace.emplace(nlohmann::json{
            {"iteration", trace_iteration_++}, {"timestamp_ns", now_ns},
            {"car", {{"position_map", {car_position.x, car_position.y}},
                     {"heading_map", {car_heading.x, car_heading.y}},
                     {"velocity_raw", {car_velocity.x, car_velocity.y}}}},
            {"pose_to_path_curvature", pose_to_path_curvature},
            {"path_map", nlohmann::json::array()},
            {"changes", nlohmann::json::array()},
            {"start_path_index", nullptr}, {"longitudinal_velocity", nullptr},
            {"accel", nullptr}, {"torque_per_wheel_nm", nullptr}
        });
        for (const auto& point : path_) (*trace)["path_map"].push_back({point.x, point.y});
    }
    const auto finish_trace = [&](const char* status) {
        if (!trace) return;
        (*trace)["status"] = status;
        trace_file_ << trace->dump() << '\n';
        trace_file_.flush(); // make completed iterations visible to the live viewer
    };
    float longitudinal_velocity;
    // TODO: this assumes path heading is tangent to the path we'll be following 
    try {
        longitudinal_velocity = car_velocity.projectOnto(car_heading);
    } catch (const std::invalid_argument& e) {
        spdlog::error("Failed to project velocity onto heading: {}", e.what());
        finish_trace("invalid_heading");
        return out;
    }
    if (trace) (*trace)["longitudinal_velocity"] = longitudinal_velocity;

    if (longitudinal_velocity < 0.0f) {
        spdlog::error("Longitudinal velocity is negative, car might be moving backwards: {}", longitudinal_velocity);
        finish_trace("negative_longitudinal_velocity");
        return out;
    }

    if (path_.empty()) {
        spdlog::error("Calling velocity controller on an empty path");
        finish_trace("empty_path");
        return out;
    }

    if (path_.size() < lookahead_distance_) {
        spdlog::error("Path is too short, only {} points, need at least {}", path_.size(), lookahead_distance_);
        finish_trace("short_path");
        return out;
    }

    start_point_index_= getHorizonPointIndex(state, car_position, car_heading);
    if (start_point_index_ == std::numeric_limits<size_t>::max()) {
        spdlog::error("Failed to find closest point on path");
        finish_trace("no_forward_point");
        return out;
    }
    if (trace) (*trace)["start_path_index"] = start_point_index_;

    if (start_point_index_ >= static_cast<size_t>(lookahead_distance_ * pathStaleLookaheadRatio)) {
        spdlog::error("Path is stale, we have passed by {} lookahead points", start_point_index_);
        finish_trace("stale_path");
        return out;
    }

    path_points_.resize(std::min(lookahead_distance_, path_.size() - start_point_index_));
    std::optional<float> accel = getAccel(state, pose_to_path_curvature, start_point_index_,
                                          car_position, longitudinal_velocity, trace ? &*trace : nullptr);
    if (!accel.has_value()) {
        spdlog::error("Failed to compute acceleration");
        finish_trace("calculation_failed");
        return out;
    }

    const float torque = accel.value() * kCarWeightKg * (constants::WHEEL_DIAMETER / 2.0f) / (4.0f * constants::GEARBOX_RATIO);
    out.desired_torques_nm.FL = torque;
    out.desired_torques_nm.FR = torque;
    out.desired_torques_nm.RL = torque;
    out.desired_torques_nm.RR = torque;
    if (trace) {
        (*trace)["accel"] = *accel;
        (*trace)["torque_per_wheel_nm"] = torque;
    }
    finish_trace("ok");
    return out;
}

const std::optional<float> VelocityPlanner::getAccel(const core::VehicleState& state, const float pose_to_path_curvature, size_t start_point_index, const core::xy_vec<float>& car_position, const float longitudinal_velocity, nlohmann::json* trace) {
    size_t end_point_index = std::min(start_point_index + lookahead_distance_ - 1, path_.size() - 1);
    const size_t path_length = end_point_index - start_point_index + 1;

    // Parameterize the spline by cumulative chord length in double precision.
    std::vector<double> times;
    times.reserve(path_length);
    times.push_back(0.0);

    std::vector<gte::Vector<2, double>> positions;
    positions.reserve(path_length);
    positions.push_back({path_[start_point_index].x, path_[start_point_index].y});

    // parametrize the path by cumulative distance along the path (chord length)
    for (std::size_t i = 1; i < path_length; ++i) {
        positions.push_back({path_[i + start_point_index].x, path_[i + start_point_index].y});
        const double d = std::hypot(positions[i][0] - positions[i - 1][0],
                                    positions[i][1] - positions[i - 1][1]);
        times.push_back(times.back() + d);
        if (times.back() <= times[i - 1]) {
            spdlog::error("Path points not in order or too close");
            return {};
        }
    }

    // fit the spline
    gte::NaturalCubicSpline<2, double> spline(true, positions, times);
    if (trace) {
        auto& spline_trace = (*trace)["spline"];
        spline_trace = {
            {"parameter", "cumulative_chord_length_m"},
            {"knots", times},
            {"segments", nlohmann::json::array()},
            {"samples", nlohmann::json::array()}
        };
        for (const auto& polynomial : spline.GetPolynomials()) {
            spline_trace["segments"].push_back({
                {polynomial[0][0], polynomial[0][1]},
                {polynomial[1][0], polynomial[1][1]},
                {polynomial[2][0], polynomial[2][1]},
                {polynomial[3][0], polynomial[3][1]}
            });
        }
    }

    for (size_t i = 0; i < path_length; ++i) {
        // for given spline section, get r(t), r't(t), r''(t) (IMPORTANT T IS NOT TIME)
        std::array<gte::Vector<2, double>, 3> jet{};
        // subtract index since path used in times starts at the point in front of the car (start index)
        spline.Evaluate(times[i], 2, jet.data()); // evaluate at cumul distance times[t]
        const gte::Vector<2, double>& position = jet[0];
        const gte::Vector<2, double>& path_tangent = jet[1]; // with respect to spline parameter t (cumulative distance along path)
        const gte::Vector<2, double>& change_in_tangent = jet[2];
            
        const double path_tangent_squared = path_tangent[0] * path_tangent[0] + path_tangent[1] * path_tangent[1];

        if (path_tangent_squared <= kMinPathTangent) {
            spdlog::error("Spline tangent is too small, cannot compute curvature");
            if (trace) (*trace)["failure"] = {{"reason", "small_spline_tangent"}, {"index", i}};
            return {};
        }

        const double cross = path_tangent[0] * change_in_tangent[1] - path_tangent[1] * change_in_tangent[0];
        const double curvature = cross / (path_tangent_squared * std::sqrt(path_tangent_squared));

        // sqrt(ax+ay) = mu * g where ax == 0 (ay = v*v*k) => v = sqrt(mu * g / k)
        const double v_max = std::min(static_cast<double>(max_car_velocity_),
                                      std::sqrt(static_cast<double>(kMu) * kGravity / std::abs(curvature)));
        path_points_[i] = {
            .point = {static_cast<float>(position[0]), static_cast<float>(position[1])},
            .curvature = static_cast<float>(curvature),
            .velocity = static_cast<float>(v_max),
        };
        if (trace) (*trace)["spline"]["samples"].push_back({
            {"index", i}, {"path_index", start_point_index + i}, {"t", times[i]},
            {"position", {position[0], position[1]}},
            {"tangent", {path_tangent[0], path_tangent[1]}},
            {"second_derivative", {change_in_tangent[0], change_in_tangent[1]}},
            {"curvature", curvature}, {"velocity_limit", path_points_[i].velocity}
        });
        if (trace) (*trace)["changes"].push_back({
            {"pass", "initial"}, {"index", i}, {"path_index", start_point_index + i}, {"changed", true},
            {"old_velocity", nullptr}, {"new_velocity", path_points_[i].velocity},
            {"curvature", path_points_[i].curvature}
        });
    }
    
    // run forward and backwards solver
    if (!solver(state, pose_to_path_curvature, path_length, car_position, longitudinal_velocity, true, trace)) return {};
    if (!solver(state, pose_to_path_curvature, path_length, car_position, longitudinal_velocity, false, trace)) return {};

    if (trace) {
        (*trace)["final_profile"] = nlohmann::json::array();
        for (const auto& point : path_points_) {
            (*trace)["final_profile"].push_back({
                {"position", {point.point.x, point.point.y}},
                {"curvature", point.curvature}, {"velocity", point.velocity}
            });
        }
    }

    // calculate accel from target speed at closest path point and current point (a = (v^2 - u^2) / 2d)
    const core::xy_vec<float> position{state.vehicle_position_map_frame.x, state.vehicle_position_map_frame.y}; // global frame
    const core::xy_vec<float> dist = position - path_points_[0].point;
    if (dist.length() < 1e-6f) {
        spdlog::error("Distance to closest path point is too small, cannot compute acceleration");
        if (trace) (*trace)["failure"] = {{"reason", "distance_to_first_point_too_small"}, {"distance", dist.length()}};
        return {};
    }
    const float accel = (path_points_[0].velocity * path_points_[0].velocity - longitudinal_velocity * longitudinal_velocity) / (2 * dist.length()); 
    if (trace) (*trace)["accel_calculation"] = {
        {"target_velocity", path_points_[0].velocity},
        {"current_velocity", longitudinal_velocity},
        {"distance", dist.length()}, {"accel", accel}
    };
    return std::make_optional(accel);
}

bool VelocityPlanner::solver(const core::VehicleState& state, const float pose_to_path_curvature, const int path_length, const core::xy_vec<float>& car_position, const float longitudinal_velocity, const bool forward, nlohmann::json* trace) {
    PathPoint current_point = {
        .point = car_position,
        .curvature = pose_to_path_curvature,
        .velocity = longitudinal_velocity, 
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
            if (trace) {
                const std::size_t to_index = forward ? i : static_cast<std::size_t>(path_length - 2) - i;
                (*trace)["failure"] = {
                    {"reason", "lateral_accel_limit"}, {"pass", forward ? "forward" : "backward"},
                    {"from_index", forward && i == 0 ? nlohmann::json(nullptr)
                        : nlohmann::json(forward ? i - 1 : to_index + 1)},
                    {"to_index", to_index}, {"curvature", start_point.curvature},
                    {"velocity", start_point.velocity}, {"lateral_accel", lateral_accel},
                    {"max_accel", kMaxAccel}
                };
            }
            if (!forward) { // restore state 
                std::reverse(path_points_.begin(), path_points_.end());
                path_points_.push_back(saved_point); 
            }
            return false;
        }
        const float a_long_avail = lateral_accel > kMaxAccel ?
         -kMaxAccel : std::sqrt( kMaxAccel * kMaxAccel - lateral_accel * lateral_accel); // set accel to -MAX if we're off limit

        const float v_long_avail = std::sqrt(start_point.velocity * start_point.velocity + 2 * a_long_avail * ds);
        const float new_velocity = std::min(next_point.velocity, v_long_avail);
        if (trace) {
            const std::size_t index = forward ? i : static_cast<std::size_t>(path_length - 2) - i;
            (*trace)["changes"].push_back({
                {"pass", forward ? "forward" : "backward"}, {"index", index},
                {"path_index", start_point_index_ + index},
                {"changed", new_velocity < next_point.velocity},
                {"old_velocity", next_point.velocity}, {"new_velocity", new_velocity},
                {"distance", ds}, {"curvature", start_point.curvature},
                {"lateral_accel", lateral_accel},
                {"longitudinal_accel_available", a_long_avail},
                {"velocity_allowed_by_accel", v_long_avail}
            });
        }
        next_point.velocity = new_velocity;
        start_point = next_point;
    }

    if (!forward) {
        std::reverse(path_points_.begin(), path_points_.end());
        path_points_.push_back(saved_point); 
    }
    return true;
}

} // namespace planning
