#pragma once

#include "StateTracker.hpp"
#include "dv_msgs.pb.h"

namespace perception {
  /**
   * Extracts cone positions and their color from a filtered point cloud.
   * @param scan The filtered point cloud to extract cones from
  */
  inline dv_msgs::Cones extract_cones(core::LidarPoint* scan) {
    return dv_msgs::Cones();
  }

}
