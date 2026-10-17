#pragma once

#include <random>
#include <vector>

#include <StateTracker.hpp>
#include "dv_msgs.pb.h"

namespace planning {

  /**
    Plans a drivable path through a known set of cones

    @param cones The cones to plan through, map frame
    @return The planned path, map frame
  */
  inline std::vector<core::xyz_vec<float>> plan_path(const dv_msgs::Cones& cones) {
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
