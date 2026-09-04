#include <StateTracker.hpp>
#include <PurePursuitController.h>

#include <algorithm>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>

namespace control {
namespace driverless {

std::vector<core::xy_vec<float>> PurePursuitController::getGoalPointCandidates(const std::vector<core::xy_vec<float>>& path, core::xy_vec<float> vehicle_pos, float lookahead_distance) {
    if (path.empty()) {
        return {};
    }

    std::vector<core::xy_vec<float>> goal_point_candidates;
    core::xy_vec<float> p1 = path[0];
    for (size_t i = 1; i < path.size(); ++i) {
        core::xy_vec<float> p2 = path[i];
        core::xy_vec<float> d = p2 - p1; // direction vector of the segment
        std::cout << "Checking segment: (" << p1.x << ", " << p1.y << ") to (" << p2.x << ", " << p2.y << ")\n";
        // if segment is degenerate (p1 == p2), skip it
        if (d * d == 0.0f) {
            p1 = p2;
            continue; 
        }

        // solve the quadratic equation for intersection of the circle and the line segment
        core::xy_vec<float> f = p1 - vehicle_pos;
        const float discriminant = ((d*f) * (d*f)) - (d*d) * (((f*f)) - lookahead_distance * lookahead_distance);
        std::cout << "Discriminant: " << discriminant << "\n";
        if (discriminant >= 0.0f) {
            const float t1 = (-(d*f) + sqrt(discriminant)) / (d*d);
            const float t2 = (-(d*f) - sqrt(discriminant)) / (d*d);

            // make sure the intersection points are within the segment and not on the line out of bounds
            std::cout << "Found intersection points: ";
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

core::xy_vec<float> PurePursuitController::selectGoalPoint(const std::vector<core::xy_vec<float>>& goal_point_candidates, core::xy_vec<float> vehicle_pos, core::xy_vec<float> vehicle_heading) {
    // TODO!: vehicle heading might be needed for a stable algo
    // assuming track is wide enough and we receive the path in front of the car, just get the closest one for now
    // float vehicle_x = static_cast<float>(foxglove::FrameTransform::default_instance().translation().x());
    // float vehicle_y = static_cast<float>(foxglove::FrameTransform::default_instance().translation().y());
    core::xy_vec<float> closest_point;

    auto dist = [vehicle_pos](const core::xy_vec<float>& point) {
        const float dx = point.x - vehicle_pos.x;
        const float dy = point.y - vehicle_pos.y;
        return dx * dx + dy * dy;
    };

    auto it = std::min_element(
        goal_point_candidates.begin(),
        goal_point_candidates.end(),
        [&](const auto& a, const auto& b) {
            return dist(a) < dist(b);
        }
    );
    std::cout << "Selected goal point: (" << it->x << ", " << it->y << ")\n";
    if (it != goal_point_candidates.end()) {
        closest_point = *it;
    }
    return closest_point;
}

float PurePursuitController::getCurvature(core::xy_vec<float> vehicle_pos, core::xy_vec<float> vehicle_heading, core::xy_vec<float> goal_point) {
    core::xy_vec<float> to_goal = goal_point - vehicle_pos;
    std::cout << "Vehicle pos: (" << vehicle_pos.x << ", " << vehicle_pos.y << "), Goal point: (" << goal_point.x << ", " << goal_point.y << "), To goal: (" << to_goal.x << ", " << to_goal.y << ")\n";
    float cross_product = vehicle_heading.x * to_goal.y - vehicle_heading.y * to_goal.x;
    std::cout << "Vehicle heading: (" << vehicle_heading.x << ", " << vehicle_heading.y << "), Cross product: " << cross_product << "\n";
    float sin_error_angle = cross_product / (vehicle_heading.length() * to_goal.length());
    std::cout << "Sin error angle: " << sin_error_angle << "\n";
    float curvature = 2.0f * sin_error_angle / lookahead_distance_;
    return curvature;
}

float PurePursuitController::getSteeringCommand(float curvature, float wheelbase) {
    return std::atan(curvature * wheelbase);
}

bool PurePursuitController::init() {
    // stub, needs to fetch and validate k_p, wheelbase and set lookahead
    return true;
} 

core::ControllerOutput PurePursuitController::step_controller(const core::VehicleState& in) {
    const core::xy_vec<float> vehicle_pos{in.vehicle_position_map_frame.x, in.vehicle_position_map_frame.y};
    const core::xy_vec<float> vehicle_heading{in.vehicle_heading_map_frame_unit_vector};
    std::cout << vehicle_pos.x << ", " << vehicle_pos.y << ", " << vehicle_heading.x << ", " << vehicle_heading.y << "\n";
    const std::vector<core::xy_vec<float>> path = loadPathFromCsv("path.csv"); // TEMPORARY UNTIL WE GET WORKING PLANNER
    
    core::ControllerOutput output{};
    output.out = std::monostate{};

    std::vector<core::xy_vec<float>> goal_point_candidates = getGoalPointCandidates(path, vehicle_pos, lookahead_distance_);
    if (goal_point_candidates.empty()) {
        // No valid goal points found, return zeros
        return output;
    }

    core::xy_vec<float> target = selectGoalPoint(goal_point_candidates, vehicle_pos, vehicle_heading);
    const float curvature = getCurvature(vehicle_pos, vehicle_heading, target);
    std::cout << "Curvature: " << curvature << "\n";
    const float steering_command = getSteeringCommand(curvature, wheelbase_);
    if (std::isfinite(steering_command)) {
        core::TorqueControlOut torque;
        torque.desired_torques_nm = {0.8f, 0.8f, 0.8f, 0.8f};
        
        output.desired_steering_deg =
            steering_command * 180.0f / static_cast<float>(M_PI);
        output.out = torque;
    }

    LoggingData data {
        .path = path,
        .vehicle_pos = vehicle_pos,
        .target_point = target,
        .curvature = curvature,
        .steering_command = steering_command,
    };
    setLoggingData(data);
    return output;
}

// TEMPORARY UNTIL WE GET WORKING PLANNER
std::vector<core::xy_vec<float>> PurePursuitController::loadPathFromCsv(
    const std::string& filename
) {
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

        if (!std::getline(stream, x_text, ',') ||
            !std::getline(stream, y_text)) {
            throw std::runtime_error(
                "Invalid path CSV line " + std::to_string(line_number)
            );
        }

        try {
            path.push_back({
                std::stof(x_text),
                std::stof(y_text)
            });
        } catch (const std::exception&) {
            throw std::runtime_error(
                "Invalid numeric value on CSV line " +
                std::to_string(line_number)
            );
        }
    }

    if (path.size() < 2) {
        throw std::runtime_error(
            "Pure-pursuit path requires at least two points"
        );
    }

    return path;
}

std::shared_ptr<hytech_msgs::PlannerVisualization> PurePursuitController::getLoggingData() const {
    auto msg = std::make_shared<hytech_msgs::PlannerVisualization>();
    auto vehicle_pos = hytech_msgs::Point2D();
    vehicle_pos.set_x_map_m(logging_data_.vehicle_pos.x);
    vehicle_pos.set_y_map_m(logging_data_.vehicle_pos.y);
    auto target_point = hytech_msgs::Point2D();
    target_point.set_x_map_m(logging_data_.target_point.x);
    target_point.set_y_map_m(logging_data_.target_point.y);


    auto arc = msg->mutable_planner_arc();
    arc->mutable_begin()->set_x_map_m(logging_data_.vehicle_pos.x);
    arc->mutable_begin()->set_y_map_m(logging_data_.vehicle_pos.y);
    arc->mutable_end()->set_x_map_m(logging_data_.target_point.x);
    arc->mutable_end()->set_y_map_m(logging_data_.target_point.y);
    arc->set_signed_curvature_inv_m(logging_data_.curvature);

    for (const auto& point : logging_data_.path) {
        auto path_point = msg->add_midpoints();
        path_point->set_x_map_m(point.x);
        path_point->set_y_map_m(point.y);
    }

    msg->set_lookahead_distance_m(lookahead_distance_);
    return msg;
}


} // namespace driverless
} // namespace control