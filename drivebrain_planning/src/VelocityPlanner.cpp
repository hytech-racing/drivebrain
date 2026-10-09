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
    : max_car_velocity_(max_car_velocity) {
        path_points_.resize(lookahead_distance_index_);
}

const std::vector<PathPoint>& VelocityPlanner::tick(const core::VehicleState& state, const float pose_to_path_curvature) {
    // path planner now sorts points so only leave the necessary amount
    start_point_index_ = 0;
    path_.resize(lookahead_distance_index_); 

    // add start point to cumul distance parametrization parametrization 
    std::vector<float> times;
    times.reserve(path_.size());
    times.push_back(0.0f);

    // copy points from core_xy_vec to gte::Vector<2, float>
    std::vector<gte::Vector<2, float>> positions;
    positions.reserve(path_.size());
    positions.push_back({path_[0].x, path_[0].y});

    // parametrize the path by cumulative distance along the path (chord length)
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


    for (size_t index = 0; index < path_.size(); ++index) {
        // for given spline section, get r(t), r't(t), r''(t) (IMPORTANT T IS NOT TIME)
        std::array<gte::Vector<2, float>, 3> jet{};
        spline.Evaluate(times[index], 2, jet.data()); // evaluate at cumul distance times[t]
        gte::Vector<2, float> position = jet[0];
        gte::Vector<2, float> path_tangent = jet[1]; // with respect to spline parameter t (cumulative distance along path)
        gte::Vector<2, float> change_in_tangent = jet[2];
            
        const float path_tangent_squared = path_tangent[0] * path_tangent[0] + path_tangent[1] * path_tangent[1];

        if (path_tangent_squared <= kMinPathTangent) {
            spdlog::error("Spline tangent is too small, cannot compute curvature");
            // TODO: stop this velocity planning iteration completely
        }

        const float cross = path_tangent[0] * change_in_tangent[1] - path_tangent[1] * change_in_tangent[0];
        const float curvature = cross / (path_tangent_squared * std::sqrt(path_tangent_squared));

        // sqrt(ax+ay) = mu * g where ax == 0 (ay = v*v*k) => v = sqrt(mu * g / k)
        const float v_max = std::min(max_car_velocity_, std::sqrt(kMu * kGravity / std::abs(curvature)));
        path_points_[index] = {
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
        path_points_.pop_back(); // avoid assigning speed to first point (as it's our car in forward pass with actual non-estimated speed)
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

} // namespace planning
