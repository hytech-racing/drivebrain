#pragma once

#include <vector>

#include <StateTracker.hpp>
#include "dv_msgs.pb.h"


// left side = blue
// right side = yellow

namespace planning {

  /**
    Plans a drivable path through a known set of cones

    @param cones The cones to plan through, map frame
    @return The planned path, map frame
  */

  // Returns the next set of coordinates for the car to follow as a path
  std::vector<core::xyz_vec<float>> plan_path(const dv_msgs::Cones& cones);

} // planning namespace
