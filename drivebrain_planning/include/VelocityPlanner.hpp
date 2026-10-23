#pragma once

#include "Controller.hpp"
#include <StateTracker.hpp>
#include <FoxgloveServer.hpp>

#include "dv_msgs.pb.h"

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <optional>
#include <vector>

#include <boost/signals2.hpp>


namespace planning {

struct PathPoint {
    core::xy_vec<float> point;
    float curvature;                  // signed, 1/m
    float velocity;             // m/s
};

class VelocityPlanner {
public:
    // max_velocity: velocity cap applied to every path point (m/s)
    // mu: tire-road friction coefficient for the friction circle
    // path_points_ahead: planning horizon in path points, including the start point
    // path_stale_ratio: path is flagged stale once the car has passed this fraction of path_points_ahead
    struct config {
        float max_velocity;
        float mu;
        int path_points_ahead;
        float path_stale_ratio;
    };

    bool init();

    /**
    Main method picking the closest forward point on path, estimating velocity limits from curvature, running solvers and returning torque request.
    @param state The current vehicle state.
    @param pose_to_path_curvature Pure Pursuit arc curvature that will be followed to keep the car on the path.
    @return The speed control output to send to the vehicle.
     */
    core::SpeedControlOut step_controller(const core::VehicleState& state, const float pose_to_path_curvature);
    
    void set_path(const std::vector<core::xy_vec<float>>& path);

    /* Diagnostics of the most recent step_controller() */
    const dv_msgs::VelocityPlannerDebug& debug() const { return _debug; }

private:
    void _handle_param_updates(const std::unordered_map<std::string, core::DBParam> &new_param_map);

    std::mutex _config_mutex;
    config _config{};
    boost::signals2::scoped_connection _param_connection;

    std::vector<core::xy_vec<float>> _path;
    std::size_t _start_point_index = SIZE_MAX;
    std::size_t _end_point_index = SIZE_MAX;
    std::vector<PathPoint> _path_points;
    std::uint64_t _iteration = 0;
    dv_msgs::VelocityPlannerDebug _debug;

    /**
     * Walk the path in direction given by `forward` and assign the maximum velocity to each point based on curvature and current speed.
     * @param state The current vehicle state.
     * @param pose_to_path_curvature Pure Pursuit arc curvature that will be followed to keep the car on the path.
     * @param forward Whether to solve forward or backward.
     * @param path_length The number of points in the path ahead of the car
     * @param cfg Config snapshot taken at the start of step_controller.
     * @return True if the solver completed successfully, false if there was an error (e.g., lateral acceleration exceeded limits).
     */
    bool solver(const core::VehicleState& state, const float pose_to_path_curvature, const int path_length, const core::xy_vec<float>& car_position, const float longitudinal_velocity, const bool forward, const config& cfg);



    const std::optional<float> get_accel(const core::VehicleState& state,  const float pose_to_path_curvature, size_t start_point_index, const core::xy_vec<float>& car_position, const float longitudinal_velocity, const config& cfg);



    std::size_t get_horizon_point_index(const core::VehicleState& state, const core::xy_vec<float>& position, const core::xy_vec<float>& heading); // get path index from which we will start calculation

};

} // namespace planning
