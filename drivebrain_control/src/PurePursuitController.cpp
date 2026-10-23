#include <StateTracker.hpp>
#include <PurePursuitController.h>
#include <SteeringEstimator.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>

#include <spdlog/spdlog.h>

static constexpr float MAX_STEERING_ANGLE_RADS = 20.0f * static_cast<float>(M_PI) / 180.0f; // maximum steering angle, 20 degrees in radians

namespace control {
namespace driverless {

std::vector<core::xyz_vec<float>> PurePursuitController::get_goal_point_candidates(const std::vector<core::xyz_vec<float>>& path, core::xyz_vec<float> vehicle_pos, float lookahead_distance) {
    if (path.empty()) {
        return {};
    }

    // Check that there is a distance between each point in the path to create a segment to follow
    std::vector<core::xyz_vec<float>> goal_point_candidates;
    core::xyz_vec<float> p1 = path[0];
    for (size_t i = 1; i < path.size(); ++i) {
        core::xyz_vec<float> p2 = path[i];
        core::xyz_vec<float> d = p2 - p1; // direction vector of the segment
        //std::cout << "Checking segment: (" << p1.x << ", " << p1.y << ") to (" << p2.x << ", " << p2.y << ")\n";

        if (d.length() < 1e-6f) {
            p1 = p2;
            spdlog::error("duplicate path points");
            continue; 
        }

        // solve the quadratic equation for intersection of the circle and the line segment
        core::xyz_vec<float> f = p1 - vehicle_pos;
        const float discriminant = ((d*f) * (d*f)) - (d*d) * (((f*f)) - lookahead_distance * lookahead_distance);
        //std::cout << "Discriminant: " << discriminant << "\n";
        if (discriminant >= 0.0f) {
            const float t1 = (-(d*f) + sqrt(discriminant)) / (d*d);
            const float t2 = (-(d*f) - sqrt(discriminant)) / (d*d);

            // make sure the intersection points are within the segment and not on the line out of bounds
            // std::cout << "Found intersection points: ";
            if (t1 >= 0.0f && t1 <= 1.0f && !(goal_point_candidates.size() >= 1 && goal_point_candidates.back() == p1 + d * t1)) {// avoid adding a vertex twice if it's exactly on the circle
                goal_point_candidates.push_back(p1 + d * t1);
                            // std::cout << "t1: " << t1 << ", point: (" << (p1 + d * t1).x << ", " << (p1 + d * t1).y << "); ";

            }
            if (discriminant != 0.0f && t2 >= 0.0f && t2 <= 1.0f && !(goal_point_candidates.size() >= 1 && goal_point_candidates.back() == p1 + d * t2)) { // avoid duplicate points when discriminant is zero as well as avoid adding a vertex twice if it's exactly on the circle
                goal_point_candidates.push_back(p1 + d * t2);
                            // std::cout << "t2: " << t2 << ", point: (" << (p1 + d * t2).x << ", " << (p1 + d * t2).y << ")\n";

            }
        }
        p1 = p2;
    }
    return goal_point_candidates;
}

std::optional<size_t> PurePursuitController::select_goal_point(const std::vector<core::xyz_vec<float>>& path, const std::vector<core::xyz_vec<float>>& goal_point_candidates, core::xyz_vec<float> vehicle_pos, core::xyz_vec<float> vehicle_heading) {
    core::xyz_vec<float> closest_point;
 
    const auto path_direction = path[1] - path[0];
    auto colinearity = [vehicle_pos, vehicle_heading, path_direction](const core::xyz_vec<float>& point) {
        const core::xyz_vec<float> target_vector = point - vehicle_pos;
        auto dot = path_direction * target_vector;
        auto cos = dot / (vehicle_heading.length() * target_vector.length());
        return cos;
    };
 
    // std::cout << "Evaluating colinearity of goal points:\n";
    // for (const auto& p : goal_point_candidates) {
    //     std::cout << "Point (" << p.x << ", " << p.y << ") has colinearity: " << colinearity(p) << "\n";
    // }
 
    auto it = std::max_element(
        goal_point_candidates.begin(),
        goal_point_candidates.end(),
        [&](const auto& a, const auto& b) {
            return colinearity(a) < colinearity(b);
        }
    );
    // std::cout << "Selected goal point: (" << it->x << ", " << it->y << ")\n";
    if (it != goal_point_candidates.end()) {
        return std::distance(goal_point_candidates.begin(), it);
    }
    return {};
}

float PurePursuitController::get_curvature(core::xyz_vec<float> vehicle_pos, core::xyz_vec<float> vehicle_heading, core::xyz_vec<float> goal_point, float lookahead_distance) {
    core::xyz_vec<float> to_goal = goal_point - vehicle_pos;
    //std::cout << "Vehicle pos: (" << vehicle_pos.x << ", " << vehicle_pos.y << "), Goal point: (" << goal_point.x << ", " << goal_point.y << "), To goal: (" << to_goal.x << ", " << to_goal.y << ")\n";
    float cross_product = vehicle_heading.x * to_goal.y - vehicle_heading.y * to_goal.x;
    //std::cout << "Vehicle heading: (" << vehicle_heading.x << ", " << vehicle_heading.y << "), Cross product: " << cross_product << "\n";
    float sin_error_angle = cross_product / (vehicle_heading.length() * to_goal.length());
    //std::cout << "Sin error angle: " << sin_error_angle << "\n";
    float curvature = 2.0f * sin_error_angle / lookahead_distance;
    return curvature;
}



void PurePursuitController::_handle_param_updates(const std::unordered_map<std::string, core::DBParam> &new_param_map) {

    spdlog::info("Entering pure pursuit controller parameter updates.");

    if (auto v = core::process_param_update<float>(new_param_map, "purepursuitcontroller/lookahead_distance")) {
        std::unique_lock lk(_config_mutex);
        _config.lookahead_distance = *v;
    }

    if (auto v = core::process_param_update<float>(new_param_map, "purepursuitcontroller/wheelbase")) {
        std::unique_lock lk(_config_mutex);
        _config.wheelbase = *v;
    }

    if (auto v = core::process_param_update<float>(new_param_map, "purepursuitcontroller/lookahead_base")) {
        std::unique_lock lk(_config_mutex);
        _config.lookahead_base = *v;
    }

    if (auto v = core::process_param_update<float>(new_param_map, "purepursuitcontroller/lookahead_gain_s")) {
        std::unique_lock lk(_config_mutex);
        _config.lookahead_gain_s = *v;
    }

    spdlog::info("Exiting pure pursuit controller parameter updates.");
}

bool PurePursuitController::init() {
    auto opt_lookahead_distance = core::FoxgloveServer::instance().get_param<float>("purepursuitcontroller/lookahead_distance");
    auto opt_wheelbase = core::FoxgloveServer::instance().get_param<float>("purepursuitcontroller/wheelbase");
    // auto opt_lookahead_base = core::FoxgloveServer::instance().get_param<float>("purepursuitcontroller/lookahead_base");
    // auto opt_lookahead_gain_s = core::FoxgloveServer::instance().get_param<float>("purepursuitcontroller/lookahead_gain_s");

    bool all_loaded = true;
    if (!opt_lookahead_distance) { spdlog::error("Missing param: PurePursuitController/lookahead_distance"); all_loaded = false; }
    if (!opt_wheelbase)          { spdlog::error("Missing param: PurePursuitController/wheelbase");          all_loaded = false; }
    // if (!opt_lookahead_base)     { spdlog::error("Missing param: PurePursuitController/lookahead_base");     all_loaded = false; }
    // if (!opt_lookahead_gain_s)   { spdlog::error("Missing param: PurePursuitController/lookahead_gain_s");   all_loaded = false; }

    if (!all_loaded) {
        spdlog::error("Couldn't load all params for pure pursuit controller.");
        return false;
    }

    if (opt_lookahead_distance.value() <= 1.0f) {
        spdlog::error("Lookahead distance too short");
        return false;
    }

    {
        std::unique_lock lk(_config_mutex);
        _config.lookahead_distance = opt_lookahead_distance.value();
        _config.wheelbase          = opt_wheelbase.value();
        // _config.lookahead_base     = opt_lookahead_base.value();
        // _config.lookahead_gain_s   = opt_lookahead_gain_s.value();
    }

    _param_connection = core::FoxgloveServer::instance().register_param_callback(std::bind(&PurePursuitController::_handle_param_updates, this, std::placeholders::_1));

    return true;
}

PurePursuitOutput PurePursuitController::step_controller(const PurePursuitInput& in) {
    const core::VehicleState& state = in.state;
    const std::vector<core::xyz_vec<float>>& path = in.path;
    PurePursuitOutput output{};

    config cur_config;
    {
        std::unique_lock lk(_config_mutex);
        cur_config = _config;
    }
    _debug.Clear();
    _debug.set_timestamp_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    _debug.set_lookahead_distance_m(cur_config.lookahead_distance);


    const core::xyz_vec<float> vehicle_pos{state.vehicle_position_map_frame.x, state.vehicle_position_map_frame.y};
    const core::xyz_vec<float> vehicle_heading{
        state.vehicle_heading_map_frame_unit_vector.x,
        state.vehicle_heading_map_frame_unit_vector.y
    };
    if (!std::isfinite(vehicle_heading.x) || !std::isfinite(vehicle_heading.y) ||
        vehicle_heading.length() < 1e-6f) {
        spdlog::error("Vehicle heading vector is invalid (NaN/inf or near zero), cannot compute curvature.");
        return output;
    }
    if (!std::isfinite(vehicle_pos.x) || !std::isfinite(vehicle_pos.y)) {
        spdlog::error("Vehicle position is invalid (NaN/inf), cannot compute curvature.");
        return output;
    }

    // std::cout << vehicle_pos.x << ", " << vehicle_pos.y << ", " << vehicle_heading.x << ", " << vehicle_heading.y << "\n";
    

    std::vector<core::xyz_vec<float>> goal_point_candidates = get_goal_point_candidates(path, vehicle_pos, cur_config.lookahead_distance);
    if (goal_point_candidates.empty()) {
        // No valid goal points found, return no steering command
        return output;
    }

    std::optional<size_t> goal_point_index = select_goal_point(path, goal_point_candidates, vehicle_pos, vehicle_heading);
    if (!goal_point_index.has_value()) {
        // No valid goal point selected, return no steering command
        return output;
    }
    core::xyz_vec<float> target = goal_point_candidates[goal_point_index.value()];
    const float curvature = get_curvature(vehicle_pos, vehicle_heading, target, cur_config.lookahead_distance);

    // Front-wheel steering angle in radians, clamped to the steering limit.
    const float steering_command = std::clamp(std::atan(curvature * cur_config.wheelbase),
                                              -MAX_STEERING_ANGLE_RADS, MAX_STEERING_ANGLE_RADS);
    _debug.mutable_target_point_m()->set_x(target.x);
    _debug.mutable_target_point_m()->set_y(target.y);
    _debug.set_curvature(curvature);
    _debug.set_yaw_rad(steering_command);

    const float steering_command_column_angle = _steering_estimator.steering_angle_rads_to_column_angle_degs(steering_command);
    _debug.set_column_angle_deg(steering_command_column_angle);
    output.column_angle_deg = steering_command_column_angle;
    output.curvature = curvature; // for the velocity planner
    return output;
}

// todo: off for now
// float PurePursuitController::dynamic_lookahead(float prev_vehicle_x, float prev_vehicle_y, float dt) {
//     config cur_config;
//     {
//         std::unique_lock lk(_config_mutex);
//         cur_config = _config;
//     }

//     core::veh_vec<float> total_torque = core::StateTracker::instance().vehicle_state().first.current_torques_nm;
//     float torque_sum = total_torque.FL + total_torque.FR + total_torque.RL + total_torque.RR;
//     float total_force = torque_sum / _wheel_radius;

//     auto dv_state = core::StateTracker::instance().vehicle_sim_pos();
//     float dx = dv_state.vehicle_x - prev_vehicle_x;
//     float dy = dv_state.vehicle_y - prev_vehicle_y;
//     float current_velocity = std::sqrt(dx * dx + dy * dy) / dt; // replace with sensor measurements?

//     float lookahead_distance = cur_config.lookahead_base + cur_config.lookahead_gain_s * current_velocity;

//     return lookahead_distance;

// }

} // namespace driverless
} // namespace control
