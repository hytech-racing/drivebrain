#include "VelocityPlanner.hpp"
#include "Literals.hpp"
#include <Mathematics/NaturalCubicSpline.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <optional>
#include <spdlog/spdlog.h>
#include <limits>

namespace planning {

constexpr float kGravity = 9.80665f;
constexpr float kMinPathTangent = 1e-6f;
const float kCarWeightKg = 180.0f;
const float pathStaleLookaheadRatio = 0.3f; // if we haven't gotten an update from path planner and we passed by 0.3 * lookahead distance points, flag it

using Status = dv_msgs::VelocityPlannerDebug;

static void set_xy(hytech_msgs::xyz_vector* vec, double x, double y) {
    vec->set_x(static_cast<float>(x));
    vec->set_y(static_cast<float>(y));
}

VelocityPlanner::VelocityPlanner(float max_car_velocity)
    : max_car_velocity_(max_car_velocity) {
        path_points_.reserve(lookahead_distance_);
}

void VelocityPlanner::setMaxCarVelocity(float velocity_mps) {
    max_car_velocity_ = velocity_mps;
}

void VelocityPlanner::setMu(float mu) {
    mu_ = mu;
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

core::SpeedControlOut VelocityPlanner::step_controller(const core::VehicleState& state,
                                                        const float pose_to_path_curvature) {
    // slow down completely with 1/4 of max torque
    core::SpeedControlOut out = {};
    out.desired_rpms = {0.0f, 0.0f, 0.0f, 0.0f}; 
    out.torque_lim_nm = {2.5f, 2.5f, 2.5f, 2.5f}; // 1/4 of max regen torque

    const core::xy_vec<float> car_position{state.vehicle_position_map_frame.x, state.vehicle_position_map_frame.y}; // global frame
    const core::xy_vec<float> car_velocity{state.current_body_vel_ms.x, state.current_body_vel_ms.y}; // TODO: currently global frame but VNData is body frame need to add transforms to drivebrain
    const core::xy_vec<float> car_heading{state.vehicle_heading_map_frame_unit_vector.x, state.vehicle_heading_map_frame_unit_vector.y}; // global frame

    debug_.Clear();
    debug_.set_timestamp_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    debug_.set_iteration(iteration_++);
    debug_.set_mu(mu_);
    debug_.set_max_car_velocity_mps(max_car_velocity_);
    set_xy(debug_.mutable_car_position_m(), car_position.x, car_position.y);
    set_xy(debug_.mutable_car_heading(), car_heading.x, car_heading.y);
    set_xy(debug_.mutable_car_velocity_raw_mps(), car_velocity.x, car_velocity.y);

    // check all variables we need
    if (car_heading.length() < 1e-6f) {
        spdlog::error("car heading is zero, can't proceed with velocity planning");
        debug_.set_status(Status::INVALID_HEADING);
        return out;
    }
    if (path_.empty()) {
        spdlog::error("Path is empty, can't proceed with velocity planning");
        debug_.set_status(Status::EMPTY_PATH);
        return out;
    }


    // TODO: by taking longitudinal velocity along heading, we assume our heading is along pure-pursuit given path
    // heading validated above, so projectOnto can't throw
    const float longitudinal_velocity = car_velocity.projectOnto(car_heading);
    debug_.set_longitudinal_velocity_mps(longitudinal_velocity);

    if (longitudinal_velocity < -0.01f) { // avoid VNData/GSS jitter so set threshold to -0.01 m/s
        spdlog::error("Longitudinal velocity is negative, car might be moving backwards: {}", longitudinal_velocity);
        debug_.set_status(Status::NEGATIVE_LONGITUDINAL_VELOCITY);
        return out;
    }

    start_point_index_= getHorizonPointIndex(state, car_position, car_heading);
    if (start_point_index_ == std::numeric_limits<size_t>::max()) {
        spdlog::error("Failed to find closest point on path");
        debug_.set_status(Status::NO_FORWARD_POINT);
        return out;
    }

    if (start_point_index_ >= static_cast<size_t>(lookahead_distance_ * pathStaleLookaheadRatio)) {
        spdlog::error("Path is stale, we have passed by {} lookahead points", start_point_index_);
        debug_.set_status(Status::STALE_PATH);
        return out;
    }

    path_points_.resize(std::min(lookahead_distance_, path_.size() - start_point_index_));
    std::optional<float> accel = getAccel(state, pose_to_path_curvature, start_point_index_,
                                          car_position, longitudinal_velocity);
    if (!accel.has_value()) {
        spdlog::error("Failed to compute acceleration"); // getAccel set the status
        return out;
    }

    const float torque = accel.value() * kCarWeightKg * (constants::WHEEL_DIAMETER / 2.0f) / (4.0f * constants::GEARBOX_RATIO);
    // TODO: clamp torque to max DTI 
    const float rpm_target = torque > 0 ? 20000.0f : 0.0f;
    out.torque_lim_nm = {torque, torque, torque, torque};
    out.desired_rpms = {rpm_target, rpm_target, rpm_target, rpm_target};

    debug_.set_accel_mps2(*accel);
    debug_.set_torque_per_wheel_nm(torque);
    debug_.set_status(Status::OK);
    return out;
}

const std::optional<float> VelocityPlanner::getAccel(const core::VehicleState& state, const float pose_to_path_curvature, size_t start_point_index, const core::xy_vec<float>& car_position, const float longitudinal_velocity) {
    size_t end_point_index = std::min(start_point_index + lookahead_distance_ - 1, path_.size() - 1);
    const size_t path_length = end_point_index - start_point_index + 1;
    if (path_length < 3) {
        spdlog::error("Need at least 3 path points for the spline");
        debug_.set_status(Status::SPLINE_TOO_FEW_POINTS);
        return {};
    }

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
        if (std::abs(times.back() - times[i - 1]) < 1e-6) {
            spdlog::error("Path points too close");
            debug_.set_status(Status::PATH_POINTS_TOO_CLOSE);
            return {};
        }
    }

    // fit the spline
    const auto startDerivative = (positions[1] - positions[0]) / times[1];
    const auto endDerivative = (positions[path_length - 1] - positions[path_length - 2]) / (times[path_length - 1] - times[path_length - 2]);

    gte::NaturalCubicSpline<2, double> spline(positions, times, startDerivative, endDerivative);
    for (const auto& polynomial : spline.GetPolynomials()) {
        auto* segment = debug_.mutable_spline()->add_segments();
        for (const auto& coefficient : polynomial) set_xy(segment->add_coefficients(), coefficient[0], coefficient[1]);
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

        if (path_tangent_squared <= 1e-6) {
            spdlog::error("Spline tangent is too small, cannot compute curvature. Spline fit might be bad.");
            debug_.set_status(Status::SMALL_SPLINE_TANGENT);
            return {};
        }

        const double cross = path_tangent[0] * change_in_tangent[1] - path_tangent[1] * change_in_tangent[0];
        const double curvature = cross / (path_tangent_squared * std::sqrt(path_tangent_squared));

        // sqrt(ax+ay) = mu * g where ax == 0 (ay = v*v*k) => v = sqrt(mu * g / k)
        const double v_max = std::min(static_cast<double>(max_car_velocity_),
                                      std::sqrt(static_cast<double>(mu_) * kGravity / std::abs(curvature)));
        path_points_[i] = {
            .point = {static_cast<float>(position[0]), static_cast<float>(position[1])},
            .curvature = static_cast<float>(curvature),
            .velocity = static_cast<float>(v_max),
        };
        auto* debug_point = debug_.add_path_points();
        set_xy(debug_point->mutable_position_m(), position[0], position[1]);
        debug_point->set_curvature(path_points_[i].curvature);
        debug_point->set_velocity_mps(path_points_[i].velocity);
    }

    // run forward and backwards solver
    if (!solver(state, pose_to_path_curvature, path_length, car_position, longitudinal_velocity, true)) return {};
    if (!solver(state, pose_to_path_curvature, path_length, car_position, longitudinal_velocity, false)) return {};
    debug_.set_target_velocity_mps(path_points_[0].velocity);

    // calculate accel from target speed at closest path point and current point (a = (v^2 - u^2) / 2d)
    const core::xy_vec<float> position{state.vehicle_position_map_frame.x, state.vehicle_position_map_frame.y}; // global frame
    const core::xy_vec<float> dist = position - path_points_[0].point;
    float accel;
    if (dist.length() < 1e-6f) {
        spdlog::error("Distance to closest path point is too small, cannot compute acceleration");
        accel = 0;
        debug_.set_status(Status::CAR_TOO_CLOSE_TO_FIRST_POINT);
        return {};
    } else {
        accel = (path_points_[0].velocity * path_points_[0].velocity - longitudinal_velocity * longitudinal_velocity) / (2 * dist.length());  
    }
    
    const float max_accel = mu_ * kGravity * 1.01f;
    accel = std::max(-max_accel, accel); // you can't control current car's speed so car-to-first-segment braking isn't enforced by solver
    return std::make_optional(accel);
}

bool VelocityPlanner::solver(const core::VehicleState& state, const float pose_to_path_curvature, const int path_length, const core::xy_vec<float>& car_position, const float longitudinal_velocity, const bool forward) {
    const float max_accel = mu_ * kGravity * 1.01f;
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
        auto& next_point = path_points_[i]; // end of current segment
        auto cur_to_next = (next_point.point - start_point.point); // vector from start to end of segment
        const float ds = cur_to_next.length(); 
        const float lateral_accel = std::abs(start_point.curvature) * start_point.velocity * start_point.velocity; // a_y = v^2 * k
        float a_long_avail; // longitudinal acceleration throughout segment TBD

        // handle already excessive accel
        if (lateral_accel > max_accel) { // lateral accel always positive 
            spdlog::error("Lateral acceleration exceeds maximum limit, previous point might've been too far. Curvature: {}, Velocity: {}, Lateral Accel: {}, Max Accel: {}", start_point.curvature, start_point.velocity, lateral_accel, max_accel);
            // TODO: 0 vs max negative VS break
            a_long_avail = -max_accel; // set accel to -max_accel since our entire limit is consumed by lateral forces and hope we don't crash
            
        } else {
            a_long_avail = std::sqrt(max_accel * max_accel - lateral_accel * lateral_accel); // friction circle model
        }

        const float v_long_avail = std::sqrt(start_point.velocity * start_point.velocity + 2 * a_long_avail * ds);
        const float new_velocity = std::min(next_point.velocity, v_long_avail);
        auto* change = debug_.add_changes();
        change->set_pass(forward ? dv_msgs::VelocityPlannerChange::FORWARD : dv_msgs::VelocityPlannerChange::BACKWARD);
        change->set_index(forward ? i : static_cast<std::size_t>(path_length - 2) - i);
        change->set_changed(new_velocity < next_point.velocity);
        change->set_new_velocity_mps(new_velocity);
        change->set_curvature(start_point.curvature);
        change->set_lateral_accel_mps2(lateral_accel);
        change->set_longitudinal_accel_available_mps2(a_long_avail);
        change->set_velocity_allowed_by_accel_mps(v_long_avail);
        change->set_lateral_accel_exceeded(lateral_accel > max_accel);
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
