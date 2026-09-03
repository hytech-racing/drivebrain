#pragma once

#include <cmath>
#include <foxglove/FrameTransform.pb.h>
#include <random>
#include <vector>
#include <algorithm>
#include <numbers>

#include <StateTracker.hpp>
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

  // Returns the next set of coordinates for the car to follow as a path
  inline std::vector<core::xyz_vec<float>> plan_path(const dv_msgs::Cones& cones) {
    
    /* Initializations */
    std::vector<core::xyz_vec<float>> path_points; // The final set of points to be followed

    std::vector<core::xyz_vec<float>> midpoints; // Set of midpoints generated based on range filter


    std::vector<double> coords; // Cone coordinates used to make the delaunay triangulation

    float max_range_squared = 100.0f;

    // FrameTransform is actually the position of the lidar relative to the map frame but the lidar is technically mounted towards the front of the car
    float vehicle_x = static_cast<float>(foxglove::FrameTransform::default_instance().translation().x());
    float vehicle_y = static_cast<float>(foxglove::FrameTransform::default_instance().translation().y());
    
    // Using FRD over FLU to avoid sign flips/conversions
    double vehicle_yaw = hytech_msgs::EkfState::default_instance().yaw_vehicle_frd_rad();

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

      //std::abs(angle_diff) < pi/2.0 &&

      // check for positive and negative angles
      if ( relative_distance < max_range_squared) {
        coords.push_back(cone.position().x());
        coords.push_back(cone.position().y());
        index_map.push_back(original_index);
      }
      ++original_index;
    }

    if (coords.size() < 3) {
      spdlog::error("not enough cones");
      return path_points; // Min 3 points required to do triangulation
    }


    delaunator::Delaunator delaunay(coords); // Triangulation occurs on construction

    std::size_t invalid_index = static_cast<std::size_t>(-1);

    for (std::size_t i = 0; i < delaunay.triangles.size(); i++) {

      // If the edge is a boundary (-1) or a twin edge already iterated over, skip it
      if (delaunay.halfedges[i] == invalid_index || delaunay.halfedges[i] < i) {
        continue; // double check if it should be > i or < i
      }

      std::size_t curr_edge = delaunay.triangles[i];
      std::size_t twin_edge = delaunay.triangles[delaunay.halfedges[i]];

      auto curr_edge_color = cones.cones().at(index_map[curr_edge]).color();
      auto twin_edge_color = cones.cones().at(index_map[twin_edge]).color();
      
      bool is_crossing_edge = (curr_edge_color == dv_msgs::Cones_ConeColor_BLUE && twin_edge_color == dv_msgs::Cones_ConeColor_YELLOW) ||
          (curr_edge_color == dv_msgs::Cones_ConeColor_YELLOW && twin_edge_color == dv_msgs::Cones_ConeColor_BLUE);

      if (is_crossing_edge) {
        float mx = (delaunay.coords[2* curr_edge] + delaunay.coords[2* twin_edge]) / (2.0);
        float my = (delaunay.coords[2*curr_edge + 1] + delaunay.coords[2* twin_edge + 1]) / (2.0);
        midpoints.push_back({mx, my, 0.0f});
      }
    }

    float min_distance = 100.0f;

    std::sort(midpoints.begin(), midpoints.end(), 
    [&](const auto& a, const auto& b) {
        float ax = a.x - vehicle_x;
        float ay = a.y - vehicle_y;

        float bx = b.x - vehicle_x;
        float by = b.y - vehicle_y;

        float dist_a = (ax * ax) + (ay * ay);
        float dist_b = (bx * bx) + (by * by);

        return dist_a < dist_b;
    });

    for (const auto& midpoint : midpoints) {
      path_points.push_back({midpoint.x, midpoint.y, 0.0f});
    }

    return path_points;
    
  }
}
