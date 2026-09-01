#pragma once

#include <foxglove/FrameTransform.pb.h>
#include <random>
#include <vector>
#include <algorithm>
#include <numbers>

#include <StateTracker.hpp>
#include "dv_msgs.pb.h"
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
    path_points.clear();
    
    if (cones.cones_size() < 3) {
      spdlog::error("not enough cones");
      return path_points; // Min 3 points required to do triangulation
    }

    std::vector<core::xyz_vec<float>> midpoints; // Set of midpoints generated based on range filter
    midpoints.clear();

    std::vector<double> coords; // Cone coordinates used to make the delaunay triangulation
    coords.clear();

    float px = 0.0f;
    float py = 0.0f;

    float max_range = 100.0f;

    float vehicle_x = static_cast<float>(foxglove::FrameTransform::default_instance().translation().x());
    float vehicle_y = static_cast<float>(foxglove::FrameTransform::default_instance().translation().y());
    

    coords.reserve(cones.cones_size()*2); // Allocate enough memory to store the x and y coordinates of each cone

    // Store cones in message order, so point index i corresponds to cones().at(i)
    for (const auto& cone : cones.cones()) {

      px = cone.position().x() - vehicle_x;
      py = cone.position().y() - vehicle_y;
      float magnitude = (px * px) + (py * py);
      std::vector<float> point_to_vehicle = {px, py}; // vector between the 
      float vector_angle = atan2(point_to_vehicle[1], point_to_vehicle[0]); // heading angle between the point and the car

      if (vector_angle < pi/3.0 && magnitude < max_range) {
        coords.push_back(cone.position().x());
        coords.push_back(cone.position().y());
      }
    }

    delaunator::Delaunator delaunay(coords); // Triangulation occurs on construction

    std::size_t invalid_index = static_cast<std::size_t>(-1);

    for (std::size_t i = 0; i < delaunay.triangles.size(); i++) {

      // If the edge is a boundary (-1) or a twin edge already iterated over, skip it
      if (delaunay.halfedges[i] == invalid_index || delaunay.halfedges[i] > i) {
        continue;
      }

      std::size_t curr_edge = delaunay.triangles[i];
      std::size_t twin_edge = delaunay.triangles[delaunay.halfedges[i]];

      auto curr_edge_color = cones.cones().at(curr_edge).color();
      auto twin_edge_color = cones.cones().at(twin_edge).color();
      
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
      [&](const auto& point_a, const auto& point_b){
        float ax = point_a.x - vehicle_x;
        float ay = point_a.y - vehicle_y;
        float bx = point_b.x - vehicle_x;
        float by = point_b.y - vehicle_y;

        float dist_a = (ax * ax) + (ay * ay);
        float dist_b = (bx * bx) + (by * by);

        return dist_a < dist_b;

      });


    for (const auto& midpoint : midpoints) {
      path_points.push_back({midpoint.x, midpoint.y, 0.0f});
    }

      
    // for (const auto& midpoint : midpoints) {
    //   float mp_to_veh_x = midpoint.x - vehicle_x;
    //   float mp_to_veh_y = midpoint.y - vehicle_y;

    //   float mp_to_veh = (mp_to_veh_x * mp_to_veh_x) + (mp_to_veh_y * mp_to_veh_y);
    //   if (mp_to_veh < min_distance) {
    //     min_distance = mp_to_veh;
    //     path_points.push_back({midpoint.x, midpoint.y, 0.0f});
    //   }
    // }



    /* Below is a hardcoded, randomized set of points that renders a small path in Foxglove.
      The logic for Delauany triangulation should live in here (and operate on @cones). The end result should be a vector of xyz_vec.
      (In all cases, z should always be zero, since this is 2D Delauany. it's not like the car is gonna fly or anything.)
    */
    // static thread_local std::mt19937 rng{std::random_device{}()};
    // std::uniform_real_distribution<float> lateral(-3.0f, 3.0f);


    // return {
    //   {0.0f, 0.0f, 0.0f},
    //   {3.0f, lateral(rng), 0.0f},
    //   {6.0f, lateral(rng), 0.0f},
    //   {9.0f, lateral(rng), 0.0f}
    // };

    return path_points;
    
  }
}
