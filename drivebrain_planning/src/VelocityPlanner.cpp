#include "VelocityPlanner.hpp"

#include <Mathematics/NaturalCubicSpline.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <spdlog/spdlog.h>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace planning {

constexpr float kGravity = 9.80665f;
constexpr float kLateralLimit = 2.0f * kGravity;
constexpr float kForwardLimit = 1.5f * kGravity;
constexpr float kBrakingLimit = 2.0f * kGravity;
constexpr float MIN_PATH_TANGENT = 1e-6f;


VelocityPlanner::VelocityPlanner(std::string path_filename, float max_car_velocity)
    : path_(loadPathFromCsv(path_filename)), max_car_velocity_(max_car_velocity) {
    if (!std::isfinite(max_car_velocity_) || max_car_velocity_ <= 0.0f) {
        throw std::invalid_argument("Maximum car velocity must be positive and finite");
    }
}

std::size_t VelocityPlanner::getHorizonPointIndex(const core::VehicleState& state) {
    // assumes valid (finite, nonzero heading)
    const core::xy_vec<float> position{state.vehicle_position_map_frame.x, state.vehicle_position_map_frame.y};
    const core::xy_vec<float> heading{state.vehicle_heading_map_frame_unit_vector};

    // first time
    if (start_point_index_ == SIZE_MAX) {
        std::size_t closest = path_.size();
        float closest_distance_squared = std::numeric_limits<float>::infinity();
        for (std::size_t i = 0; i < path_.size(); ++i) {
            if (path_[i] * heading < 0.0f) {
                continue;
            }
            if ((path_[i] - position).length() < closest_distance_squared) {
                closest_distance_squared = (path_[i] - position).length();
                closest = i;
            }
        }
        if (closest == path_.size()) {
            spdlog::error("No path point is ahead of the vehicle");
        }
        return start_point_index_;
    }

    // if we know where we were before, advance from there until we start going further from the car
    std::size_t closest = start_point_index_;
    float closest_distance_squared = (path_[start_point_index_] - position).length();

    // iterate with offset to allow for wraparound
    for (std::size_t offset = 1; offset < path_.size(); ++offset) {
        const std::size_t index = (start_point_index_ + offset) % path_.size();
        const float distance_squared = (path_[index] - position).length();
        if (distance_squared > closest_distance_squared) {
            break;
        }
        closest = index;
        closest_distance_squared = distance_squared;
    }

    return start_point_index_;
}

const std::vector<VelocitySample>& VelocityPlanner::tick(const core::VehicleState& state) {
    // obtain start point for velocity planner
    start_point_index_ = getHorizonPointIndex(state);

    // cumulative (polyline, chord-length) distance for spline parametrization
    std::vector<float> times;
    times.reserve(path_.size());
    times.push_back(0.0f);

    // copy points from core_xy_vec to gte::Vector<2, float>
    std::vector<gte::Vector<2, float>> positions;
    positions.reserve(path_.size());
    positions.push_back({path_[0].x, path_[0].y});

    for (std::size_t i = 1; i < path_.size(); ++i) {
        positions.push_back({path_[i].x, path_[i].y});
        const float dx = path_[i].x - path_[i - 1].x;
        const float dy = path_[i].y - path_[i - 1].y;
        times.push_back(times.back() + std::hypot(dx, dy));
        if (times.back() <= times[i - 1]) {
            spdlog::error("Path points not in order or too close");
        }
    }

    // fit the spline
    gte::NaturalCubicSpline<2, float> spline(true, positions, times);


    std::vector<VelocitySample> samples;

    size_t iterations = 0;

    while (iterations < lookahead_distance_index_) {
        size_t t = (start_point_index_ + iterations) % times.size();
        iterations++;

        // for given spline section, get r(t), r't(t), r''(t) (IMPORTANT T IS NOT TIME)
        std::array<gte::Vector<2, float>, 3> jet{};
        spline.Evaluate(t, 2, jet.data());
        gte::Vector<2, float> position = jet[0];
        gte::Vector<2, float> path_tangent = jet[1]; // with respect to spline parameter t (cumulative distance along path)
        gte::Vector<2, float> change_in_tangent = jet[2];
            
        const float path_tangent_squared = path_tangent[0] * path_tangent[0] + path_tangent[1] * path_tangent[1];

        if (path_tangent_squared <= MIN_PATH_TANGENT) {
            spdlog::error("Spline tangent is too small, cannot compute curvature");
        }

    //     const float cross = jet[1][0] * jet[2][1] - jet[1][1] * jet[2][0];
    //     const float curvature = cross / (speed_squared * std::sqrt(speed_squared));
    //     if (!std::isfinite(curvature)) {
    //         throw std::runtime_error("Spline curvature is invalid");
    //     }
    //     const float curvature_speed_limit = std::abs(curvature) <= kDerivativeEpsilon
    //         ? max_car_velocity_
    //         : std::sqrt(kLateralLimit / std::abs(curvature));
    //     const float velocity_limit = std::min(max_car_velocity_, curvature_speed_limit);

    //     // The friction ellipse leaves this fraction of longitudinal acceleration.
    //     const float lateral_accel = velocity_limit * velocity_limit * std::abs(curvature);
    //     const float lateral_fraction = std::min(1.0f, lateral_accel / kLateralLimit);
    //     const float longitudinal_fraction = std::sqrt(std::max(0.0f,
    //         1.0f - lateral_fraction * lateral_fraction));
    //     samples.push_back({{jet[0][0], jet[0][1]}, curvature, velocity_limit,
    //                        -kBrakingLimit * longitudinal_fraction,
    //                        kForwardLimit * longitudinal_fraction});
    }

    // velocity_samples_ = std::move(samples);
    // return velocity_samples_;
}

std::vector<core::xy_vec<float>> VelocityPlanner::loadPathFromCsv(
    const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Unable to open path CSV: " + filename);
    }

    std::vector<core::xy_vec<float>> path;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(file, line)) {
        ++line_number;
        if (line.empty()) {
            continue;
        }
        std::stringstream stream(line);
        std::string x_text;
        std::string y_text;
        if (!std::getline(stream, x_text, ',') || !std::getline(stream, y_text)) {
            throw std::runtime_error("Invalid path CSV line " + std::to_string(line_number));
        }
        try {
            const core::xy_vec<float> point{std::stof(x_text), std::stof(y_text)};
            if (!std::isfinite(point.x) || !std::isfinite(point.y)) {
                throw std::runtime_error("Non-finite path coordinate");
            }
            if (!path.empty() && point == path.back()) {
                throw std::runtime_error("Duplicate consecutive path point");
            }
            path.push_back(point);
        } catch (const std::exception&) {
            throw std::runtime_error("Invalid numeric value on CSV line " +
                                     std::to_string(line_number));
        }
    }
    if (path.size() < 3) {
        throw std::runtime_error("C2 cubic spline requires at least three path points");
    }
    return path;
}
} // namespace planning
