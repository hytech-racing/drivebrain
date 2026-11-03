#include "PathPlanner.hpp"

#include <cmath>
#include <foxglove/FrameTransform.pb.h>
#include <limits>
#include <random>
#include <vector>
#include <algorithm>
#include <numbers>
#include <StateTracker.hpp>
#include "base_msgs.pb.h"
#include "dv_msgs.pb.h"
#include "hytech_msgs.pb.h"
#include <delaunator.hpp>
#include <FoxgloveServer.hpp>

// left side = blue
// right side = yellow

constexpr float MAX_TRACK_WIDTH_SQ = 36.0f; // 6 meters^2
constexpr float MIN_TRACK_WIDTH_SQ = 2.8f * 2.8f; // rules requires track width to be at least 3 meters
constexpr float MIN_COS_TURN = -0.3f;      // only reject points that are clearly behind
constexpr float MAX_JUMP_SQ  = 8.0f * 8.0f; 

// TODO: make max track width something that changes with event OR a tunable param in foxglove since accel and skidpad have different track widths

namespace planning {

// Returns the next set of coordinates for the car to follow as a path
std::vector<core::xyz_vec<float>> plan_path(const dv_msgs::Cones& cones) {

    /* Initializations */
    std::vector<core::xyz_vec<float>> midpoints; // Set of points to follow
    std::vector<double> coords; // Cone coordinates used to make the delaunay triangulation
    const auto vehicle_state = core::StateTracker::instance().vehicle_state().first;
    const float vehicle_x = vehicle_state.vehicle_position_map_frame.x;
    const float vehicle_y = vehicle_state.vehicle_position_map_frame.y;
    const float heading_x = vehicle_state.vehicle_heading_map_frame_unit_vector.x;
    const float heading_y = vehicle_state.vehicle_heading_map_frame_unit_vector.y;


    coords.reserve(cones.cones_size()*2); // Allocate enough memory to store the x and y coordinates of each cone

    // Store cones in message order, so point index i corresponds to cones().at(i)
    for (const auto& cone : cones.cones()) {
        coords.push_back(cone.position().x());
        coords.push_back(cone.position().y());
    }

    if (coords.size() < 6) { // Each cone has an x and y
        spdlog::error("not enough cones");
        return midpoints; // Min 3 points required to do triangulation
    }

    delaunator::Delaunator delaunay(coords); // Triangulation occurs on construction

    // half edges are shared edges between triangles 
    // indices that do not point to the index of a corresponding half edge store -1
    std::size_t invalid_index = static_cast<std::size_t>(-1); 

    for (std::size_t i = 0; i < delaunay.triangles.size(); i++) {

        // If the edge is a boundary (-1) or a twin edge already iterated over, skip it
        if (delaunay.halfedges[i] == invalid_index || delaunay.halfedges[i] < i) {
        continue; 
        }

        std::size_t curr_edge = delaunay.triangles[i];
        std::size_t twin_edge = delaunay.triangles[delaunay.halfedges[i]];

        auto curr_edge_color = cones.cones().at(curr_edge).color();
        auto twin_edge_color = cones.cones().at(twin_edge).color();

        float dx = delaunay.coords[2 * curr_edge] - delaunay.coords[2 * twin_edge];
        float dy = delaunay.coords[2 * curr_edge + 1] - delaunay.coords[2 * twin_edge +1];
        float edge_length_sq = (dx * dx) + (dy * dy);
        
        if (edge_length_sq > MAX_TRACK_WIDTH_SQ || edge_length_sq < MIN_TRACK_WIDTH_SQ) continue;
        
        bool is_true_crossing_edge = (curr_edge_color == dv_msgs::Cones_ConeColor_BLUE && twin_edge_color == dv_msgs::Cones_ConeColor_YELLOW) ||
             (curr_edge_color == dv_msgs::Cones_ConeColor_YELLOW && twin_edge_color == dv_msgs::Cones_ConeColor_BLUE);

        bool uncolored_edges = curr_edge_color == dv_msgs::Cones_ConeColor_UNKNOWN || twin_edge_color == dv_msgs::Cones_ConeColor_UNKNOWN;


        // // only looking for edges that cross the width of the track - this means that one end is a blue cone and the other end is a yellow cone
        // bool is_crossing_edge = (curr_edge_color == dv_msgs::Cones_ConeColor_BLUE && twin_edge_color == dv_msgs::Cones_ConeColor_YELLOW) ||
        //     (curr_edge_color == dv_msgs::Cones_ConeColor_YELLOW && twin_edge_color == dv_msgs::Cones_ConeColor_BLUE);

        // bool color_unknown = curr_edge_color == dv_msgs::Cones_ConeColor_UNKNOWN; // WIP

        // if (is_crossing_edge) {
        // float dx = delaunay.coords[2 * curr_edge] - delaunay.coords[2 * twin_edge];
        // float dy = delaunay.coords[2 * curr_edge + 1] - delaunay.coords[2 * twin_edge +1];
        // float edge_length_sq = (dx * dx) + (dy * dy);

        // // filtering crossing edges based on length - they should only be the width of the track
        // if (edge_length_sq > MAX_TRACK_WIDTH_SQ || edge_length_sq < MIN_TRACK_WIDTH_SQ) continue;
        
        float mx = (delaunay.coords[2* curr_edge] + delaunay.coords[2* twin_edge]) / (2.0);
        float my = (delaunay.coords[2*curr_edge + 1] + delaunay.coords[2* twin_edge + 1]) / (2.0);
        midpoints.push_back({mx, my, 0.0f});
    
    }

    if (midpoints.empty()) return {};

    // Find the first closest midpoint to the car
    float closest_ahead = std::numeric_limits<float>::max();
    std::size_t start = midpoints.size();

    for (std::size_t i = 0; i < midpoints.size(); i++) {
        float dx = midpoints[i].x - vehicle_x;
        float dy = midpoints[i].y - vehicle_y;
        float d = dx * dx + dy * dy;
        
        // using the dot product to determine whether a point is in front of or behind the car
        if (dx * heading_x + dy * heading_y > 0.0f && d < closest_ahead) {
        closest_ahead = d;
        start = i;
        }
    }

    if (start == midpoints.size()) return {}; // if no midpoints are close to the car, plan no path

    std::swap(midpoints[0], midpoints[start]); // make the index of the closest cone the start of the midpoints

    // sort the rest of the midpoints based on proximity to the first one
    std::size_t count = 1;
    float hx = heading_x;
    float hy = heading_y;

    while (count < midpoints.size()) {
        const auto current = midpoints[count - 1];
        std::size_t best_next = midpoints.size();
        float best_cost = std::numeric_limits<float>::max(); // using a cost variable to evaluate vector distance from the first midpoint

        for (std::size_t j = count; j < midpoints.size(); j++) {
        float dx = midpoints[j].x - current.x;
        float dy = midpoints[j].y - current.y;
        float d_square = dx * dx + dy * dy;

        if (d_square < 1e-6f || d_square > MAX_JUMP_SQ) continue;                       

        float dist = std::sqrt(d_square);
        float angle = (dx * hx + dy * hy) / dist;
        if (angle <= MIN_COS_TURN) continue; 
        float cost = dist * (2.0f - angle) + color_weight; // angle = 1 means straight ahead, then cost is only based on distance
        if (uncolred_edges) {
            cost *= 2; // prioritize colored over uncolored edges
        }
        if (cost < best_cost) {
            best_cost = cost;
            best_next = j;
        }
        }

        if (best_next == midpoints.size()) break;

        std::swap(midpoints[count], midpoints[best_next]);

        float dx = midpoints[count].x - current.x;
        float dy = midpoints[count].y - current.y;
        float dist = std::sqrt(dx * dx + dy * dy);
        hx = dx / dist; 
        hy = dy / dist;
        ++count;
    }

    midpoints.resize(count); // need to resize based on the number of valid ones found from the original batch
    return midpoints;

} // plan_path

}
