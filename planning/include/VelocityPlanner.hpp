#pragma once

#include <StateTracker.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace planning {

struct PathPoint {
    core::xy_vec<float> point;
    float curvature;                  // signed, 1/m
    float velocity;             // m/s
    float longitudinal_accel;     // m/s^2
};

class VelocityPlanner {
public:
    explicit VelocityPlanner(std::string path_filename = "path.csv",
                             float max_car_velocity = 20.0f);

    // Run one planning cycle. The returned samples begin at the nearest point ahead.
    const std::vector<PathPoint>& tick(const core::VehicleState& state);
    
private:
    std::vector<core::xy_vec<float>> path_;
    std::size_t start_point_index_ = SIZE_MAX;
    std::size_t end_point_index_ = SIZE_MAX;
    std::size_t lookahead_distance_index_ = 10; // 10 points ahead from start
    float max_car_velocity_ = 20.0f; // m/s
    std::vector<PathPoint> path_points_;

    void solver(const core::VehicleState& state, bool forward = true);
    std::size_t getHorizonPointIndex(const core::VehicleState& state);
    static std::vector<core::xy_vec<float>> loadPathFromCsv(const std::string& filename);
};

} // namespace planning
