#pragma once

#include <StateTracker.hpp>

#include <cstddef>
#include <string>
#include <vector>
inline constexpr std::size_t PATH_POINTS_AHEAD = 20;


namespace planning {

struct PathPoint {
    core::xy_vec<float> point;
    float curvature;                  // signed, 1/m
    float velocity;             // m/s
    float longitudinal_accel;     // m/s^2
};

class VelocityPlanner {
public:
    explicit VelocityPlanner(float max_car_velocity = 20.0f);

    // Run one planning cycle. The returned samples begin at the nearest point ahead.
    const std::vector<PathPoint>& tick(const core::VehicleState& state,  const float pose_to_path_curvature);
    
    void setPath(const std::vector<core::xy_vec<float>>& path) {
        path_ = path;
    }

private:
    std::vector<core::xy_vec<float>> path_;
    std::size_t start_point_index_ = SIZE_MAX;
    std::size_t end_point_index_ = SIZE_MAX;
    std::size_t lookahead_distance_index_ = PATH_POINTS_AHEAD; // 10 points ahead from start
    float max_car_velocity_ = 20.0f; // m/s
    std::vector<PathPoint> path_points_;

    /**
     * Walk the path in direction given by `forward` and assign the maximum velocity to each point based on curvature and current speed.
     * @param state The current vehicle state.
     * @param pose_to_path_curvature Pure Pursuit arc curvature that will be followed to keep the car on the path.
     * @param forward Whether to solve forward or backward.
     */
    void solver(const core::VehicleState& state, const float pose_to_path_curvature, bool forward = true);
};

} // namespace planning
