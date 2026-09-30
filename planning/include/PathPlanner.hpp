#pragma once

#include <cmath>
#include <foxglove/FrameTransform.pb.h>
#include <random>
#include <vector>
#include <algorithm>
#include <numbers>

#include <StateTracker.hpp>
#include "base_msgs.pb.h"
#include "dv_msgs.pb.h"
#include "hytech_msgs.pb.h"
#include <delaunator.hpp>

// left side = blue
// right side = yellow


namespace planning {

  /**
    Plans a drivable path through a known set of cones
    
    @param cones The cones to plan through, map frame
    @return The planned path, map frame
  */  


  inline constexpr double pi = 3.14159265358979323846; 
  inline constexpr float MAX_TRACK_WIDTH_SQ = 5.0f * 5.0f; // track width changes based on event - need a way to monitor event type (accel vs skidpad check rules)
  inline constexpr float MAX_LOOKAHEAD_RANGE_SQ = 200.0f;

  // Returns the next set of coordinates for the car to follow as a path
  inline std::vector<core::xyz_vec<float>> plan_path(const dv_msgs::Cones& cones) {
    
    /* Initializations */
    std::vector<core::xyz_vec<float>> midpoints; // Set of points to follow

    std::vector<double> coords; // Cone coordinates used to make the delaunay triangulation

    // Position and orientation updating only uses sim zmq message - need to update to work with sensor data for real-car validation!!
    float vehicle_x = core::StateTracker::instance().vehicle_sim_pos().vehicle_x;
    float vehicle_y = core::StateTracker::instance().vehicle_sim_pos().vehicle_y;

    float w = core::StateTracker::instance().vehicle_sim_pos().orientation_w;
    float x = core::StateTracker::instance().vehicle_sim_pos().orientation_x;
    float y = core::StateTracker::instance().vehicle_sim_pos().orientation_y;
    float z = core::StateTracker::instance().vehicle_sim_pos().orientation_z;


    // Finding yaw from quaternion orientation
    float sin_yaw = 2.0 * (w * z + x * y);
    float cos_yaw = 1.0 - 2.0 * (y * y + z * z);
    float vehicle_yaw = std::atan2(sin_yaw, cos_yaw);

    coords.reserve(cones.cones_size()*2); // Allocate enough memory to store the x and y coordinates of each cone
    std::vector<std::size_t> index_map; 
    index_map.reserve(cones.cones_size());

    std::size_t original_index = 0;

    // Store cones in message order, so point index i corresponds to cones().at(i)
    for (const auto& cone : cones.cones()) {

      float px = cone.position().x() - vehicle_x;
      float py = cone.position().y() - vehicle_y;
      float relative_distance = (px * px) + (py * py);
      float angle_diff = std::atan2(py, px) - vehicle_yaw; //calculated in radians

      // Restrict the angle range to be between -pi and +pi
      while (angle_diff > pi) {
        angle_diff -= 2.0f * pi; // subtract 2*pi (2 rad) to get back within the range (too positive of an angle)
      }
      
      while (angle_diff < (-1)*pi) {
        angle_diff += 2.0f * pi; // add 2*pi to get into the range (too negative of an angle)
      }


      // only keep cone coordinates that are in front of the car - can add a max range filter to see how far ahead the filtering should look at 
      if (std::abs(angle_diff) < pi/2 && relative_distance < MAX_LOOKAHEAD_RANGE_SQ) {
        coords.push_back(cone.position().x());
        coords.push_back(cone.position().y());
        index_map.push_back(original_index);
      }
      ++original_index;
    }

    
    if (coords.size() < 6) { // Each cone has an x and y
      spdlog::error("not enough cones");
      return midpoints; // Min 3 points required to do triangulation
    }

    delaunator::Delaunator delaunay(coords); // Triangulation occurs on construction

    std::size_t invalid_index = static_cast<std::size_t>(-1); // indices that do not point to the index of a corresponding half edge store -1

    for (std::size_t i = 0; i < delaunay.triangles.size(); i++) {

      // If the edge is a boundary (-1) or a twin edge already iterated over, skip it
      if (delaunay.halfedges[i] == invalid_index || delaunay.halfedges[i] < i) {
        continue; // double check if it should be > i or < i
      }

      std::size_t curr_edge = delaunay.triangles[i];
      std::size_t twin_edge = delaunay.triangles[delaunay.halfedges[i]];

      auto curr_edge_color = cones.cones().at(index_map[curr_edge]).color();
      auto twin_edge_color = cones.cones().at(index_map[twin_edge]).color();
      
      // only looking for edges that cross the width of the track
      bool is_crossing_edge = (curr_edge_color == dv_msgs::Cones_ConeColor_BLUE && twin_edge_color == dv_msgs::Cones_ConeColor_YELLOW) ||
          (curr_edge_color == dv_msgs::Cones_ConeColor_YELLOW && twin_edge_color == dv_msgs::Cones_ConeColor_BLUE);

      if (is_crossing_edge) {
        float dx = delaunay.coords[2 * curr_edge] - delaunay.coords[2 * twin_edge];
        float dy = delaunay.coords[2 * curr_edge + 1] - delaunay.coords[2 * twin_edge +1];

        // filtering crossing edges based on length - they should only be the width of the track
        if ((dx * dx) + (dy * dy) <= MAX_TRACK_WIDTH_SQ) {
          float mx = (delaunay.coords[2* curr_edge] + delaunay.coords[2* twin_edge]) / (2.0);
          float my = (delaunay.coords[2*curr_edge + 1] + delaunay.coords[2* twin_edge + 1]) / (2.0);
          midpoints.push_back({mx, my, 0.0f});
        }
      }
    }
    
    if (midpoints.empty()) return midpoints;

    // Finding the first closest midpoint
    float closest_ahead = std::numeric_limits<float>::max();
    std::size_t start;

    for (std::size_t i = 0; i < midpoints.size(); i++) {
      float dx = midpoints[i].x - vehicle_x;
      float dy = midpoints[i].y - vehicle_y;
      float d = dx * dx + dy * dy;

      if (dx * cos_yaw + dy * sin_yaw > 0.0f && d < closest_ahead) {
        closest_ahead = d;
        start = i;
      }
    }

    std::swap(midpoints[0], midpoints[start]);
    for (std::size_t i = 0; i + 1 < midpoints.size(); i++) {
      const auto current = midpoints[i];

      auto next = std::min_element(midpoints.begin() + i + 1, midpoints.end(), [&](const auto& a, const auto& b) {
        return ((a.x - current.x) * (a.x - current.x) + (a.y - current.y) * (a.y - current.y))< ((b.x - current.x) * (b.x - current.x) + (b.y - current.y) * (b.y - current.y)); 
      });

      std::swap(*(midpoints.begin() + i + 1), *next);

    }

    return midpoints;
    
  }
}

