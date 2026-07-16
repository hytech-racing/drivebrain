#pragma once

#include "StateTracker.hpp"
#include "dv_msgs.pb.h"

namespace perception {
  /**
    Performs ground and object filtering on a LiDAR scan
    @param scan The pointcloud to run filtering on
  */
   inline void filter_cloud(core::LidarPoint* scan) {
     return;
   }
}
