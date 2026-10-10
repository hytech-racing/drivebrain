#pragma once

#include "Controller.hpp"
#include <StateTracker.hpp>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>
inline constexpr std::size_t kPathPointsAhead = 20;


namespace planning {

struct PathPoint {
    core::xy_vec<float> point;
    float curvature;                  // signed, 1/m
    float velocity;             // m/s
};

class VelocityPlanner {
public:
    explicit VelocityPlanner(float max_car_velocity = 20.0f);

    /**
    Main method picking the closest forward point on path, estimating velocity limits from curvature, running solvers and returning torque request.
    @param state The current vehicle state.
    @param pose_to_path_curvature Pure Pursuit arc curvature that will be followed to keep the car on the path.
    @return The torque control output to send to the vehicle.
     */
    core::TorqueControlOut step_controller(const core::VehicleState& state,  const float pose_to_path_curvature);
    
    void setPath(const std::vector<core::xy_vec<float>>& path) {
        path_ = path;
    }


private:
    std::vector<core::xy_vec<float>> path_;
    std::size_t start_point_index_ = SIZE_MAX;
    std::size_t end_point_index_ = SIZE_MAX;
    std::size_t lookahead_distance_ = kPathPointsAhead; // 20 points ahead including start
    float max_car_velocity_ = 10.0f; // m/s
    std::vector<PathPoint> path_points_;

    /**
     * Walk the path in direction given by `forward` and assign the maximum velocity to each point based on curvature and current speed.
     * @param state The current vehicle state.
     * @param pose_to_path_curvature Pure Pursuit arc curvature that will be followed to keep the car on the path.
     * @param forward Whether to solve forward or backward.
     * @param path_length The number of points in the path ahead of the car
     * @return True if the solver completed successfully, false if there was an error (e.g., lateral acceleration exceeded limits).
     */
    bool solver(const core::VehicleState& state, const float pose_to_path_curvature, const int path_length, bool forward = true);


    const std::optional<float> getAccel(const core::VehicleState& state,  const float pose_to_path_curvature, size_t start_point_index);


    std::size_t getHorizonPointIndex(const core::VehicleState& state); // get path index from which we will start calculation

};

} // namespace planning
