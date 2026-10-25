#include "PathPlanner.hpp"

#include <cmath>
#include <limits>
#include <vector>
#include <algorithm>

#include <spdlog/spdlog.h>
// delaunator v0.4.0 defines non-inline functions in its header, so it must only be included in one TU
#include <delaunator.hpp>
#include "FoxgloveServer.hpp"

namespace {

//auto MAX_TRACK_WIDTH_SQ = core::FoxgloveServer::instance().get_param<float>("PathPlanning/max_track_width");
//auto MIN_COS_TURN = core::FoxgloveServer::instance().get_param<float>("PathPlanning/min_cos_turn");
constexpr float MAX_TRACK_WIDTH_SQ = 36.0f;
constexpr float MIN_TRACK_WIDTH_SQ = 2.8f * 2.8f; // a crossing edge spans at least a track width (3 m by the rules, less SLAM noise)
constexpr float MIN_COS_TURN = -0.3f;      // only reject points that are behind the car
constexpr float MAX_COS_CROSSING = 0.75f;  // a crossing edge is within ~41 deg of square to the heading
constexpr float STRADDLE_FACTOR = 1.0f;    // the heading line passes within this fraction of the edge's lateral extent of its midpoint
constexpr float UNCOLOURED_COST_FACTOR = 1.5f; // blue-yellow pairs win over edges with an uncoloured end at similar cost

// just to simulate the lidar's max field of view
constexpr float MAX_JUMP_SQ  = 8.0f * 8.0f;
constexpr std::size_t PATH_POINTS_AHEAD = 20;

// Whether an edge (yellow-to-blue vector when a colour is known) with midpoint at (dx, dy) from a point heading (hx, hy)
// can be the next track crossing: the heading line passes between its ends, it is near square to the heading, known colour on its side
bool crosses_ahead(const core::xyz_vec<float>& edge, int known_ends, float dx, float dy, float hx, float hy) {
  if (std::abs(hx * dy - hy * dx) >= STRADDLE_FACTOR * std::abs(hx * edge.y - hy * edge.x)) return false;
  if (std::abs(hx * edge.x + hy * edge.y) > MAX_COS_CROSSING * std::hypot(edge.x, edge.y)) return false;
  return known_ends == 0 || -hy * edge.x + hx * edge.y > 0.0f;
}

} // namespace

namespace planning {

  std::vector<core::xyz_vec<float>> plan_path(const dv_msgs::Cones& cones) {
      /* Initializations */
      std::vector<core::xyz_vec<float>> midpoints; // Set of points to follow
      std::vector<double> coords; // Cone coordinates used to make the delaunay triangulation
      std::vector<core::xyz_vec<float>> edges; // each midpoint's edge, from the yellow/right end to the blue/left end when known
      std::vector<int> known_ends; // per midpoint: 2 = blue-yellow pair, 1 = one coloured end, 0 = none

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
        // coloured ends (orange counts as uncoloured); two cones of one known colour never cross the track
        const int known = (curr_edge_color == dv_msgs::Cones_ConeColor_BLUE || curr_edge_color == dv_msgs::Cones_ConeColor_YELLOW) +
            (twin_edge_color == dv_msgs::Cones_ConeColor_BLUE || twin_edge_color == dv_msgs::Cones_ConeColor_YELLOW);

        // only looking for edges that cross the width of the track - this means that one end is a blue cone and the other end is a yellow cone
        bool is_crossing_edge = (curr_edge_color == dv_msgs::Cones_ConeColor_BLUE && twin_edge_color == dv_msgs::Cones_ConeColor_YELLOW) ||
            (curr_edge_color == dv_msgs::Cones_ConeColor_YELLOW && twin_edge_color == dv_msgs::Cones_ConeColor_BLUE);

        if (is_crossing_edge || known < 2) {
          float dx = delaunay.coords[2 * curr_edge] - delaunay.coords[2 * twin_edge];
          float dy = delaunay.coords[2 * curr_edge + 1] - delaunay.coords[2 * twin_edge +1];
          float edge_length_sq = (dx * dx) + (dy * dy);

          // filtering crossing edges based on length - they should only be the width of the track
          // track width changes with event type (autocross vs accel) so add a parameter to check this??
          if (edge_length_sq > MAX_TRACK_WIDTH_SQ) continue;
          if (edge_length_sq < MIN_TRACK_WIDTH_SQ) continue;

          float mx = (delaunay.coords[2* curr_edge] + delaunay.coords[2* twin_edge]) / (2.0);
          float my = (delaunay.coords[2*curr_edge + 1] + delaunay.coords[2* twin_edge + 1]) / (2.0);
          midpoints.push_back({mx, my});
          const bool flip = curr_edge_color == dv_msgs::Cones_ConeColor_YELLOW || twin_edge_color == dv_msgs::Cones_ConeColor_BLUE;
          edges.push_back({flip ? -dx : dx, flip ? -dy : dy, 0.0f});
          known_ends.push_back(known);

        }
      }

      if (midpoints.empty()) return {};

      // Find the first closest midpoint to the car
      float closest_ahead = std::numeric_limits<float>::max();
      std::size_t start = midpoints.size();

      for (std::size_t i = 0; i < midpoints.size(); i++) {
        float dx = midpoints[i].x - vehicle_x;
        float dy = midpoints[i].y - vehicle_y;
        float d = dx * dx + dy * dy;
        if (known_ends[i] < 2 && !crosses_ahead(edges[i], known_ends[i], dx, dy, heading_x, heading_y)) continue; // blue-yellow pairs as before
        if (known_ends[i] < 2) d *= UNCOLOURED_COST_FACTOR * UNCOLOURED_COST_FACTOR; // prefer starting on a blue-yellow pair

        // using the dot product to determine whether a point is in front of or behind the car
        if (dx * heading_x + dy * heading_y > 0.0f && d < closest_ahead) {
          closest_ahead = d;
          start = i;
        }
      }

      if (start == midpoints.size()) return {}; // if no midpoints are close to the car, plan no path

      std::swap(midpoints[0], midpoints[start]); // make the index of the closest cone the start of the midpoints
      std::swap(edges[0], edges[start]);
      std::swap(known_ends[0], known_ends[start]);

      // sort the rest of the midpoints based on proximity to the one closest to the car
      // midpoints[count - 1] = the last point you added to the path so far
      // midpoints [count] = the slot that you are looking to fill
      std::size_t count = 1;
      float hx = heading_x;
      float hy = heading_y;
      while (count < midpoints.size() && count < PATH_POINTS_AHEAD) {
        const auto current = midpoints[count - 1];
        std::size_t best_next = midpoints.size();
        float best_cost = std::numeric_limits<float>::max(); // using a cost variable to evaluate vector distance from the first midpoint

        for (std::size_t j = count; j < midpoints.size(); j++) {
          float dx = midpoints[j].x - current.x;
          float dy = midpoints[j].y - current.y;
          float d_square = dx * dx + dy * dy;

          if (d_square < 1e-6f || d_square > MAX_JUMP_SQ) continue;
          if (known_ends[j] < 2 && !crosses_ahead(edges[j], known_ends[j], dx, dy, hx, hy)) continue; // blue-yellow pairs as before

          float dist = std::sqrt(d_square);
          float angle = (dx * hx + dy * hy) / dist;
          if (angle <= MIN_COS_TURN) continue;
          float cost = dist * (2.0f - angle); // angle = 1 means straight ahead, then cost is only based on distance
          if (known_ends[j] < 2) cost *= UNCOLOURED_COST_FACTOR; // blue-yellow pairs first when both are there
          if (cost < best_cost) {
            best_cost = cost;
            best_next = j;
          }
        }

        if (best_next == midpoints.size()) break;

        std::swap(midpoints[count], midpoints[best_next]);
        std::swap(edges[count], edges[best_next]);
        std::swap(known_ends[count], known_ends[best_next]);

        // update the heading based on the point just added so that the points follow the shape of the path, not just a straight line from the first midpoint
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
} // planning namespace
