#pragma once

#include <StateTracker.hpp>

#include <cstddef>
#include <string>
#include <vector>

namespace planning {

struct VelocitySample {
    core::xy_vec<float> point;
    float curvature;                  // signed, 1/m
    float velocity_limit;             // m/s
    float min_longitudinal_accel;     // m/s^2
    float max_longitudinal_accel;     // m/s^2
};

class VelocityPlanner {
public:
    explicit VelocityPlanner(std::string path_filename = "path.csv",
                             float max_car_velocity = 20.0f);

    // Run one planning cycle. The returned samples begin at the nearest point ahead.
    const std::vector<VelocitySample>& tick(const core::VehicleState& state);


    std::size_t getHorizonPointIndex(const core::VehicleState& state);
    const std::vector<core::xy_vec<float>>& path() const { return path_; }
    const std::vector<VelocitySample>& velocitySamples() const { return velocity_samples_; }
    std::size_t horizonPointIndex() const { return start_point_index_; }

private:
    static std::vector<core::xy_vec<float>> loadPathFromCsv(const std::string& filename);
    std::vector<float> velocityEvaluationTimes(const std::vector<float>& path_times,
                                                std::size_t first_index) const;

    std::vector<core::xy_vec<float>> path_;
    std::vector<VelocitySample> velocity_samples_;
    std::size_t start_point_index_ = SIZE_MAX;
    std::size_t end_point_index_ = SIZE_MAX;
    std::size_t lookahead_distance_index_ = 10; // 10 points ahead from start
    float max_car_velocity_ = 20.0f; // m/s
};

} // namespace planning
