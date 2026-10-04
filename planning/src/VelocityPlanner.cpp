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
constexpr float kMinPathTangent = 1e-6f;
const float kCarWeightKg = 180.0f;
const float kMu = 1.5;
const float kMaxAccel = kMu * kGravity;

VelocityPlanner::VelocityPlanner(std::string path_filename, float max_car_velocity)
    : path_(loadPathFromCsv(path_filename)), max_car_velocity_(max_car_velocity) {
        path_points_.resize(lookahead_distance_index_);
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
        return closest;
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

    return closest;
}

const std::vector<PathPoint>& VelocityPlanner::tick(const core::VehicleState& state, const float pose_to_path_curvature) {
    // obtain start point for velocity planner
    start_point_index_ = getHorizonPointIndex(state);

    // cumulative (polyline, chord-length) distance for spline parametrization
    std::vector<float> times;
    times.resize(path_.size());
    times.push_back(0.0f);

    // copy points from core_xy_vec to gte::Vector<2, float>
    std::vector<gte::Vector<2, float>> positions;
    positions.resize(path_.size());
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


    std::vector<PathPoint> samples;

    size_t iterations = 0;

    while (iterations < lookahead_distance_index_) {
        size_t index = (start_point_index_ + iterations) % times.size();
        iterations++;

        // for given spline section, get r(t), r't(t), r''(t) (IMPORTANT T IS NOT TIME)
        std::array<gte::Vector<2, float>, 3> jet{};
        spline.Evaluate(times[index], 2, jet.data()); // evaluate at cumul distance times[t]
        gte::Vector<2, float> position = jet[0];
        gte::Vector<2, float> path_tangent = jet[1]; // with respect to spline parameter t (cumulative distance along path)
        gte::Vector<2, float> change_in_tangent = jet[2];
            
        const float path_tangent_squared = path_tangent[0] * path_tangent[0] + path_tangent[1] * path_tangent[1];

        if (path_tangent_squared <= kMinPathTangent) {
            spdlog::error("Spline tangent is too small, cannot compute curvature");
        }

        const float cross = path_tangent[0] * change_in_tangent[1] - path_tangent[1] * change_in_tangent[0];
        const float curvature = cross / (path_tangent_squared * std::sqrt(path_tangent_squared));

        // sqrt(ax+ay) = mu * g where ax == 0 (ay = v*v*k) => v = sqrt(mu * g / k)
        const float v_max = std::min(max_car_velocity_, std::sqrt(kMu * kGravity / std::abs(curvature)));
        path_points_[iterations] = {
            .point = {position[0], position[1]},
            .curvature = curvature,
            .velocity = v_max,
            .longitudinal_accel = -1,
        };
    }

    solver(state, true); // forward solve with curvature limits
    solver(state, false); // backward solve with accel limits
    

}

void VelocityPlanner::solver(const core::VehicleState& state, const float pose_to_path_curvature, bool forward) {
    const core::xy_vec<float> position{state.vehicle_position_map_frame.x, state.vehicle_position_map_frame.y}; // global frame
    const core::xy_vec<float> velocity{state.current_body_vel_ms.x, state.current_body_vel_ms.y}; // local frame
    const float speed = velocity.length();

    PathPoint current_point = {
        .point = position,
        .curvature = pose_to_path_curvature,
        .velocity = speed, 
        .longitudinal_accel = -1
    };

    PathPoint start_point;
    PathPoint saved_point;
    if (forward) {
        start_point = current_point;
    } else {
        start_point = path_points_.back();
        saved_point = path_points_.back();
        path_points_.pop_back();
        std::reverse(path_points_.begin(), path_points_.end());
    }

    for (auto& next_point: path_points_) {
        auto v = (next_point.point - start_point.point);
        const float ds = v.length();
        const float lateral_accel = std::abs(start_point.curvature) * start_point.velocity * start_point.velocity; // a_y = v^2 * k
        if (lateral_accel > kMaxAccel) { // always positive no need for abs()
            spdlog::error("Lateral acceleration exceeds maximum limit, previous point might've been too far");
        }
        const float a_long_avail = lateral_accel > kMaxAccel ?
         -kMaxAccel : std::sqrt( kMaxAccel * kMaxAccel - lateral_accel * lateral_accel); // set accel to -MAX if we're off limit

        const float v_long_avail = std::sqrt(start_point.velocity * start_point.velocity + 2 * a_long_avail * ds);
        next_point.velocity = std::min(next_point.velocity, v_long_avail);
        start_point = next_point;
    }

    if (!forward) {
        std::reverse(path_points_.begin(), path_points_.end());
        path_points_.push_back(saved_point); 
    }
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
