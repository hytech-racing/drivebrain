#pragma once

#include <random>
#include <vector>

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
    
    std::vector<double> coords;
    coords.reserve(cones.cones_size()*2); //allocate enough memory to store the x and y coordinates of each cone


    // TODO: pre process the data to alternate between yellow and blue cone coordinates

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

    }

    
    // adding each cone coordinate to a 2D vector needed to construct a delaunator object
    for (auto& cone : cones.cones()) {
      coords.push_back(cone.position().x()); 
      coords.push_back(cone.position().y());
      // not adding the z vector since that's assumed to be 0 
    }

    delaunator::Delaunator delaunay(coords); // triangulation happens on construction

    




  //const ::hytech_msgs::xyz_vector cone_position = cones.cones().at(0).position();


    /* Below is a hardcoded, randomized set of points that renders a small path in Foxglove.
      The logic for Delauany triangulation should live in here (and operate on @cones). The end result should be a vector of xyz_vec.
      (In all cases, z should always be zero, since this is 2D Delauany. it's not like the car is gonna fly or anything.)
    */
    static thread_local std::mt19937 rng{std::random_device{}()};
    std::uniform_real_distribution<float> lateral(-3.0f, 3.0f);


    return {
      {0.0f, 0.0f, 0.0f},
      {3.0f, lateral(rng), 0.0f},
      {6.0f, lateral(rng), 0.0f},
      {9.0f, lateral(rng), 0.0f}
    };
  }

}
