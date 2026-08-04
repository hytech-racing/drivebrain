#pragma once

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
    float mx = 0.0f;
    float my = 0.0f;

    std::vector<double> coords; // Cone coordinates used to make the delaunay triangulation

    coords.resize(cones.cones_size()*2); // Allocate enough memory to store the x and y coordinates of each cone

    // Pre-process the data to alternate between blue and yellow cones
    int inner_index = 0;
    int outer_index = 2;
    for (int i = 0; i < cones.cones_size(); i++) {
      if (cones.cones().at(i).color() == dv_msgs::Cones_ConeColor_BLUE) {
        coords[inner_index] = cones.cones().at(i).position().x();
        coords[inner_index + 1] = cones.cones().at(i).position().y();
        inner_index +=4;
      } 

      if (cones.cones().at(i).color() == dv_msgs::Cones_ConeColor_YELLOW) {
        coords[outer_index] = cones.cones().at(i).position().x();
        coords[outer_index + 1] = cones.cones().at(i).position().y();
        outer_index +=4;
      }

      // TODO: come up with something for orange cones (exit and entry lanes)

    }
    

    delaunator::Delaunator delaunay(coords); // Triangulation occurs on construction

    /*
      Easier way to do it without filtering - just look for crossing edges (one end is blue, other end is yellow) 
      and take the midpoint of that !!
    */

    std::size_t invalid_index = static_cast<std::size_t>(-1);

    for (int i = 0; i < delaunay.triangles.size(); i++) {

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
        mx = (delaunay.coords[2* curr_edge] + delaunay.coords[2* twin_edge]) / (2.0);
        my = (delaunay.coords[2* (curr_edge + 1)] + delaunay.coords[2* (twin_edge + 1)]) / (2.0);
      }

      path_points.push_back({mx, my, 0.0f});

    }

    // Filter out exterior triangles based on whether at least one of the vertices is a blue cone and at least one is a yellow cone 
    // std::vector<std::size_t> filtered_triangles;

    // for (std::size_t i = 0; i < delaunay.triangles.size(); i+=3) { 
    //   std::size_t a = delaunay.coords[2* delaunay.triangles[i]]; 
    //   std::size_t b = delaunay.coords[2* delaunay.triangles[i+1]];
    //   std::size_t c = delaunay.coords[2* delaunay.triangles[i+2]];

    //   bool hasBlue = (cones.cones().at(a).color() == dv_msgs::Cones_ConeColor_BLUE) || 
    //                 (cones.cones().at(b).color() == dv_msgs::Cones_ConeColor_BLUE) ||
    //                 (cones.cones().at(c).color() == dv_msgs::Cones_ConeColor_BLUE) ;

    //   bool hasYellow = (cones.cones().at(a).color() == dv_msgs::Cones_ConeColor_YELLOW) || 
    //                 (cones.cones().at(b).color() == dv_msgs::Cones_ConeColor_YELLOW) ||
    //                 (cones.cones().at(c).color() == dv_msgs::Cones_ConeColor_YELLOW) ;
      
    //   if (hasBlue && hasYellow) {
    //     filtered_triangles.push_back(i); // Keep the starting index of the triangle that is kept
    //   }
    // }    

      //TODO: come up with a filter for cones being near each other in sequence or distance based


    // Looking for inner edges to take the midpoints of
    // for (std::size_t i = 0; i < delaunay.triangles.size(); i+=3) {
    //   std::size_t twin_edge = delaunay.halfedges[i];
    //   if (std::find(filtered_triangles.begin(), filtered_triangles.end(), delaunay.triangles[i]) == filtered_triangles.end() || 
    //       std::find(filtered_triangles.begin(), filtered_triangles.end(), twin_edge) == filtered_triangles.end()) {
    //     continue;
    //   }

    //   if (delaunay.halfedges[i] != -1) {
    //     double x0 = delaunay.coords[2* delaunay.triangles[i]];
    //     double y0 = delaunay.coords[2* delaunay.triangles[i+1]];
    //     double x1 = delaunay.coords[2* delaunay.triangles[twin_edge]];
    //     double y1 = delaunay.coords[2* delaunay.triangles[twin_edge + 1]];

    //     float mx = (x0 + x1) / 2.0;
    //     float my = (y0 + y1) / 2.0;
    //     float mz = 0.0;
    //     core::xyz_vec<float> midpoint = {mx, my, mz};

    //     path_points.push_back(midpoint);
        
    //   }

    // }

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
