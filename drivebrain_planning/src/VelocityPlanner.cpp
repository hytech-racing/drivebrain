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

constexpr float GRAVITY = 9.80665f;
constexpr float MIN_PATH_TANGENT = 1e-6f;
const float CAR_WEIGHT_KG = 180.0f;

using Status = dv_msgs::VelocityPlannerDebug;

static void set_xy(hytech_msgs::xyz_vector* vec, double x, double y) {
    vec->set_x(static_cast<float>(x));
    vec->set_y(static_cast<float>(y));
}

void VelocityPlanner::_handle_param_updates(const std::unordered_map<std::string, core::DBParam> &new_param_map) {

    spdlog::info("Entering velocity planner parameter updates.");

    if (auto v = core::process_param_update<float>(new_param_map, "velocityplanner/max_velocity")) {
        std::unique_lock lk(_config_mutex);
        _config.max_velocity = *v;
    }

    if (auto v = core::process_param_update<float>(new_param_map, "velocityplanner/mu")) {
        std::unique_lock lk(_config_mutex);
        _config.mu = *v;
    }

    if (auto v = core::process_param_update<int>(new_param_map, "velocityplanner/path_points_ahead")) {
        std::unique_lock lk(_config_mutex);
        _config.path_points_ahead = *v;
    }

    if (auto v = core::process_param_update<float>(new_param_map, "velocityplanner/path_stale_ratio")) {
        std::unique_lock lk(_config_mutex);
        _config.path_stale_ratio = *v;
    }

    spdlog::info("Exiting velocity planner parameter updates.");
}

bool VelocityPlanner::init() {
    auto opt_max_velocity = core::FoxgloveServer::instance().get_param<float>("velocityplanner/max_velocity");
    auto opt_mu = core::FoxgloveServer::instance().get_param<float>("velocityplanner/mu");
    auto opt_path_points_ahead = core::FoxgloveServer::instance().get_param<int>("velocityplanner/path_points_ahead");
    auto opt_path_stale_ratio = core::FoxgloveServer::instance().get_param<float>("velocityplanner/path_stale_ratio");

    bool all_loaded = true;
    if (!opt_max_velocity)      { spdlog::error("Missing param: VelocityPlanner/max_velocity");      all_loaded = false; }
    if (!opt_mu)                { spdlog::error("Missing param: VelocityPlanner/mu");                all_loaded = false; }
    if (!opt_path_points_ahead) { spdlog::error("Missing param: VelocityPlanner/path_points_ahead"); all_loaded = false; }
    if (!opt_path_stale_ratio)  { spdlog::error("Missing param: VelocityPlanner/path_stale_ratio");  all_loaded = false; }

    if (!all_loaded) {
        spdlog::error("Couldn't load all params for velocity planner.");
        return false;
    }
    {
        std::unique_lock lk(_config_mutex);
        _config.max_velocity = opt_max_velocity.value();
        _config.mu                = opt_mu.value();
        _config.path_points_ahead = opt_path_points_ahead.value();
        _config.path_stale_ratio  = opt_path_stale_ratio.value();
    }
    _path_points.reserve(opt_path_points_ahead.value());

    _param_connection = core::FoxgloveServer::instance().register_param_callback(std::bind(&VelocityPlanner::_handle_param_updates, this, std::placeholders::_1));

    return true;
}

void VelocityPlanner::set_path(const std::vector<core::xy_vec<float>>& path) {
    std::size_t path_points_ahead;
    {
        std::unique_lock lk(_config_mutex);
        path_points_ahead = static_cast<std::size_t>(_config.path_points_ahead);
    }

    if (path.size() < path_points_ahead) {
        spdlog::error("Path is too short, only {} points, need at least {}", path.size(), path_points_ahead);
        _path.clear();
        return;
    }
    _path = path;
}

std::size_t VelocityPlanner::get_horizon_point_index(const core::VehicleState& state, const core::xy_vec<float>& position, const core::xy_vec<float>& heading) {
    // iterate with offset to allow for wraparound
    size_t index = 0;
    while (index < _path.size()) {
        const core::xy_vec<float> to_point = _path[index] - position;
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
    config cur_config;
    {
        std::unique_lock lk(_config_mutex);
        cur_config = _config;
    }

    // slow down completely with 1/4 of max torque
    core::SpeedControlOut out = {};
    out.desired_rpms = {0.0f, 0.0f, 0.0f, 0.0f}; 
    out.torque_lim_nm = {2.5f, 2.5f, 2.5f, 2.5f}; // 1/4 of max regen torque

    const core::xy_vec<float> car_position{state.vehicle_position_map_frame.x, state.vehicle_position_map_frame.y}; // global frame
    const core::xy_vec<float> car_velocity{state.current_body_vel_ms.x, state.current_body_vel_ms.y}; // TODO: currently global frame but VNData is body frame need to add transforms to drivebrain
    const core::xy_vec<float> car_heading{state.vehicle_heading_map_frame_unit_vector.x, state.vehicle_heading_map_frame_unit_vector.y}; // global frame

    _debug.Clear();
    _debug.set_timestamp_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    _debug.set_iteration(_iteration++);
    _debug.set_mu(cur_config.mu);
    _debug.set_max_car_velocity_mps(cur_config.max_velocity);
    set_xy(_debug.mutable_car_position_m(), car_position.x, car_position.y);
    set_xy(_debug.mutable_car_heading(), car_heading.x, car_heading.y);
    set_xy(_debug.mutable_car_velocity_raw_mps(), car_velocity.x, car_velocity.y);

    // check all variables we need
    if (car_heading.length() < 1e-6f) {
        spdlog::error("car heading is zero, can't proceed with velocity planning");
        _debug.set_status(Status::INVALID_HEADING);
        return out;
    }
    if (_path.empty()) {
        spdlog::error("Path is empty, can't proceed with velocity planning");
        _debug.set_status(Status::EMPTY_PATH);
        return out;
    }


    // TODO: by taking longitudinal velocity along heading, we assume our heading is along pure-pursuit given path
    // heading validated above, so projectOnto can't throw
    const float longitudinal_velocity = car_velocity.projectOnto(car_heading);
    _debug.set_longitudinal_velocity_mps(longitudinal_velocity);

    if (longitudinal_velocity < -0.01f) { // avoid VNData/GSS jitter so set threshold to -0.01 m/s
        spdlog::error("Longitudinal velocity is negative, car might be moving backwards: {}", longitudinal_velocity);
        _debug.set_status(Status::NEGATIVE_LONGITUDINAL_VELOCITY);
        return out;
    }

    _start_point_index= get_horizon_point_index(state, car_position, car_heading);
    if (_start_point_index == std::numeric_limits<size_t>::max()) {
        spdlog::error("Failed to find closest point on path");
        _debug.set_status(Status::NO_FORWARD_POINT);
        return out;
    }

    const std::size_t path_points_ahead = static_cast<std::size_t>(cur_config.path_points_ahead);
    // if we haven't gotten an update from path planner and passed by path_stale_ratio * path_points_ahead points, flag it
    if (_start_point_index >= static_cast<size_t>(path_points_ahead * cur_config.path_stale_ratio)) {
        spdlog::error("Path is stale, we have passed by {} lookahead points", _start_point_index);
        _debug.set_status(Status::STALE_PATH);
        return out;
    }

    _path_points.resize(std::min(path_points_ahead, _path.size() - _start_point_index));
    std::optional<float> accel = get_accel(state, pose_to_path_curvature, _start_point_index,
                                          car_position, longitudinal_velocity, cur_config);
    if (!accel.has_value()) {
        spdlog::error("Failed to compute acceleration"); // get_accel set the status
        return out;
    }

    const float torque = accel.value() * CAR_WEIGHT_KG * (constants::WHEEL_DIAMETER / 2.0f) / (4.0f * constants::GEARBOX_RATIO);
    // TODO: clamp torque to max DTI 
    const float rpm_target = torque > 0 ? 20000.0f : 0.0f;
    out.torque_lim_nm = {torque, torque, torque, torque};
    out.desired_rpms = {rpm_target, rpm_target, rpm_target, rpm_target};

    _debug.set_accel_mps2(*accel);
    _debug.set_torque_per_wheel_nm(torque);
    _debug.set_status(Status::OK);
    return out;
}

const std::optional<float> VelocityPlanner::get_accel(const core::VehicleState& state, const float pose_to_path_curvature, size_t start_point_index, const core::xy_vec<float>& car_position, const float longitudinal_velocity, const config& cfg) {
    size_t end_point_index = std::min(start_point_index + static_cast<std::size_t>(cfg.path_points_ahead) - 1, _path.size() - 1);
    const size_t path_length = end_point_index - start_point_index + 1;
    if (path_length < 3) {
        spdlog::error("Need at least 3 path points for the spline");
        _debug.set_status(Status::SPLINE_TOO_FEW_POINTS);
        return {};
    }

    // Parameterize the spline by cumulative chord length in double precision.
    std::vector<double> times;
    times.reserve(path_length);
    times.push_back(0.0);

    std::vector<gte::Vector<2, double>> positions;
    positions.reserve(path_length);
    positions.push_back({_path[start_point_index].x, _path[start_point_index].y});

    // parametrize the path by cumulative distance along the path (chord length)
    for (std::size_t i = 1; i < path_length; ++i) {
        positions.push_back({_path[i + start_point_index].x, _path[i + start_point_index].y});
        const double d = std::hypot(positions[i][0] - positions[i - 1][0],
                                    positions[i][1] - positions[i - 1][1]);
        times.push_back(times.back() + d);
        if (std::abs(times.back() - times[i - 1]) < 1e-6) {
            spdlog::error("Path points too close");
            _debug.set_status(Status::PATH_POINTS_TOO_CLOSE);
            return {};
        }
    }

    // fit the spline
    const auto start_derivative = (positions[1] - positions[0]) / times[1];
    const auto end_derivative = (positions[path_length - 1] - positions[path_length - 2]) / (times[path_length - 1] - times[path_length - 2]);

    gte::NaturalCubicSpline<2, double> spline(positions, times, start_derivative, end_derivative);
    for (const auto& polynomial : spline.GetPolynomials()) {
        auto* segment = _debug.mutable_spline()->add_segments();
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
            _debug.set_status(Status::SMALL_SPLINE_TANGENT);
            return {};
        }

        const double cross = path_tangent[0] * change_in_tangent[1] - path_tangent[1] * change_in_tangent[0];
        const double curvature = cross / (path_tangent_squared * std::sqrt(path_tangent_squared));

        // sqrt(ax+ay) = mu * g where ax == 0 (ay = v*v*k) => v = sqrt(mu * g / k)
        const double v_max = std::min(static_cast<double>(cfg.max_velocity),
                                      std::sqrt(static_cast<double>(cfg.mu) * GRAVITY / std::abs(curvature)));
        _path_points[i] = {
            .point = {static_cast<float>(position[0]), static_cast<float>(position[1])},
            .curvature = static_cast<float>(curvature),
            .velocity = static_cast<float>(v_max),
        };
        auto* debug_point = _debug.add_path_points();
        set_xy(debug_point->mutable_position_m(), position[0], position[1]);
        debug_point->set_curvature(_path_points[i].curvature);
        debug_point->set_velocity_mps(_path_points[i].velocity);
    }

    // run forward and backwards solver
    if (!solver(state, pose_to_path_curvature, path_length, car_position, longitudinal_velocity, true, cfg)) return {};
    if (!solver(state, pose_to_path_curvature, path_length, car_position, longitudinal_velocity, false, cfg)) return {};
    _debug.set_target_velocity_mps(_path_points[0].velocity);

    // calculate accel from target speed at closest path point and current point (a = (v^2 - u^2) / 2d)
    const core::xy_vec<float> position{state.vehicle_position_map_frame.x, state.vehicle_position_map_frame.y}; // global frame
    const core::xy_vec<float> dist = position - _path_points[0].point;
    float accel;
    if (dist.length() < 1e-6f) {
        spdlog::error("Distance to closest path point is too small, cannot compute acceleration");
        accel = 0;
        _debug.set_status(Status::CAR_TOO_CLOSE_TO_FIRST_POINT);
        return {};
    } else {
        accel = (_path_points[0].velocity * _path_points[0].velocity - longitudinal_velocity * longitudinal_velocity) / (2 * dist.length());  
    }
    
    const float max_accel = cfg.mu * GRAVITY * 1.01f;
    accel = std::max(-max_accel, accel); // you can't control current car's speed so car-to-first-segment braking isn't enforced by solver
    return std::make_optional(accel);
}

bool VelocityPlanner::solver(const core::VehicleState& state, const float pose_to_path_curvature, const int path_length, const core::xy_vec<float>& car_position, const float longitudinal_velocity, const bool forward, const config& cfg) {
    const float max_accel = cfg.mu * GRAVITY * 1.01f;
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
        start_point = _path_points.back(); // evaluate from the end
        saved_point = _path_points.back(); // save the last point to put it back after reverse
        _path_points.pop_back(); // speed at start point is given, avoid reassigning it
        std::reverse(_path_points.begin(), _path_points.end());
    }

    const int max_index = forward ? path_length : path_length - 1; // if backward, we ignore the last point but dont add fist point
    for (size_t i = 0; i < max_index; ++i) {
        auto& next_point = _path_points[i]; // end of current segment
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
        auto* change = _debug.add_changes();
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
        std::reverse(_path_points.begin(), _path_points.end());
        _path_points.push_back(saved_point); 
    }
    return true;
}

} // namespace planning
