#pragma once

#include <random>
#include <vector>
#include <algorithm>

#include <StateTracker.hpp>
#include "dv_msgs.pb.h"
#include <delaunator.hpp>

// left side = blue
// right side = yellow

/* 
  1. locate cone coordinates - written to cone.position in dv_msg.pb -- done
  2. form the delaunay triangulation -- done on construction of delaunator object
  3. remove exterior triangles & make sure all triangles have a blue and yellow cone as two of its vertices - need the middle path
  4. find the midpoints of all internal edges (blue to yellow across a path)
  5. fit a spline through the midpoints 
  6. smooth the trajectory -  compute curvature, then target speed


  Extras: 
  - either check that the z direction of cone location vectors are 0 or convert them to 0 regardless of the value
  - need to figure out a way to handle the coordinate updates - make a delaunay object everytime seems like it would be computationally expensive
*/


namespace planning {

  /**
    Plans a drivable path through a known set of cones

    @param cones The cones to plan through, map frame
    @return The planned path, map frame
  */  

  // function returns a vector of coordinates for the next target point in the path
  inline std::vector<core::xyz_vec<float>> plan_path(const dv_msgs::Cones& cones) {
    
    std::vector<core::xyz_vec<float>> path_points;
    std::vector<double> coords; // must be a double for delaunator constructor
    coords.reserve(cones.cones_size()*2); //allocate enough memory to store the x and y coordinates of each cone


    // pre-process data to alternate between yellow and blue cones
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
    

    delaunator::Delaunator delaunay(coords); // triangulation occurs on construction

    //TODO: come up with a filter for cones being near each other in sequence or distance based

    // filter out exterior triangles - check each triangle to see if it has at least one blue and yellow cone as vertices 
    std::vector<std::size_t> filtered_triangles;

    for (std::size_t i = 0; i < delaunay.triangles.size(); i+=3) { 
      std::size_t a = delaunay.coords[2* delaunay.triangles[i]]; //don't need coords?
      std::size_t b = delaunay.coords[2* delaunay.triangles[i+1]];
      std::size_t c = delaunay.coords[2* delaunay.triangles[i+2]];

      // check that at least one of the coordinates is based on a blue triangle
      bool hasBlue = (cones.cones().at(a).color() == dv_msgs::Cones_ConeColor_BLUE) || 
                    (cones.cones().at(b).color() == dv_msgs::Cones_ConeColor_BLUE) ||
                    (cones.cones().at(c).color() == dv_msgs::Cones_ConeColor_BLUE) ;

      bool hasYellow = (cones.cones().at(a).color() == dv_msgs::Cones_ConeColor_YELLOW) || 
                    (cones.cones().at(b).color() == dv_msgs::Cones_ConeColor_YELLOW) ||
                    (cones.cones().at(c).color() == dv_msgs::Cones_ConeColor_YELLOW) ;
      
      if (hasBlue && hasYellow) {
        filtered_triangles.push_back(i); // starting index of the triangle
      }
      
    }    

    for (std::size_t i = 0; i < delaunay.triangles.size(); i+=3) {
      
      // check to see if the starting index of the current triangle is one of the kept triangles
      std::size_t twin_edge = delaunay.halfedges[i]
      if (!std::find(filtered_triangles.begin(), filtered_triangles.end(), delaunay.triangles[i]) != filtered_triangles.end() || 
          !std::find(filtered_triangles.begin(), filtered_triangles.end(), twin_edge) != filtered_triangles.end()) {
        continue;
      }

      if (delaunay.halfedges[i] != -1) {
        double x0 = delaunay.coords[2* delaunay.triangles[i]];
        double y0 = delaunay.coords[2* delaunay.triangles[i+1]];
        double x1 = delaunay.coords[2* delaunay.triangles[twin_edge]];
        double y1 = delaunay.coords[2* delaunay.triangles[twin_edge + 1]];

        float mx = (x0 + x1) / 2.0;
        float my = (y0 + y1) / 2.0;
        float mz = 0.0;
        core::xyz_vec<float> midpoint = {mx, my, mz};

        path_points.push_back(midpoint);
        
        
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
