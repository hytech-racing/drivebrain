#pragma once

#include <foxglove/FrameTransform.pb.h>
#include <random>
#include <vector>
#include <algorithm>

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

  // Returns the next set of coordinates for the car to follow as a path
  inline std::vector<core::xyz_vec<float>> plan_path(const dv_msgs::Cones& cones) {
    
    std::vector<core::xyz_vec<float>> path_points; // THe final set of points to be followed

    if (cones.cones_size() < 3) return path_points; // Min 3 points required to do triangulation
  
    // could use vndata lat and lon or estimator x/y

    struct LocalCone {
      float x, y;
      dv_msgs::Cones_ConeColor color;
    };
    std::vector<LocalCone> filtered_cones;

    float mx = 0.0f;
    float my = 0.0f;

    float max_range = 15.0f;
    float half_fov_rad = static_cast<float>(M_PI)/1.5f;

    float vehicle_x = static_cast<float>(foxglove::FrameTransform::default_instance().translation().x());
    float vehicle_y = static_cast<float>(foxglove::FrameTransform::default_instance().translation().y());

    std::vector<double> coords; // Cone coordinates used to make the delaunay triangulation

    coords.reserve(cones.cones_size()*2); // Allocate enough memory to store the x and y coordinates of each cone

    // Store cones in message order, so point index i corresponds to cones().at(i)
    for (const auto& cone : cones.cones()) {

      float x = cone.position().x();
      float y = cone.position().y();
      float r = std::hypot(x, y);
      float angle = std::atan2(y, x);

      if (r <= max_range && std::fabs(angle) <= half_fov_rad) {
        filtered_cones.push_back(LocalCone{x, y, cone.color()});

      // coords.push_back(cone.position().x());
      // coords.push_back(cone.position().y());
      }

    if (filtered_cones.size() < 3) {
      return path_points;
    }
    
    for (const auto& c : filtered_cones) {
      coords.push_back(c.x);
      coords.push_back(c.y);
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

      // auto curr_edge_color = cones.cones().at(curr_edge).color();
      // auto twin_edge_color = cones.cones().at(twin_edge).color();

      auto curr_edge_color = filtered_cones.at(curr_edge).color;
      auto twin_edge_color = filtered_cones.at(twin_edge).color;
      
      bool is_crossing_edge = (curr_edge_color == dv_msgs::Cones_ConeColor_BLUE && twin_edge_color == dv_msgs::Cones_ConeColor_YELLOW) ||
          (curr_edge_color == dv_msgs::Cones_ConeColor_YELLOW && twin_edge_color == dv_msgs::Cones_ConeColor_BLUE);

      if (is_crossing_edge) {
        mx = (delaunay.coords[2* curr_edge] + delaunay.coords[2* twin_edge]) / (2.0);
        my = (delaunay.coords[2*curr_edge + 1] + delaunay.coords[2* twin_edge + 1]) / (2.0);
        path_points.push_back({mx, my, 0.0f});
      }

    }


    return path_points;



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
    }
  }
}
