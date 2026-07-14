#pragma once

#include "dv_msgs.pb.h"

namespace perception {
  /**
    Performs ground and object filtering on a LiDAR scan
    @param scan The pointcloud to run filtering on
  */
  // TODO(dv): dv_msgs::PointCloud was removed (foxglove.PointCloud is logging/sim-wire only);
  // redefine the input as the raw point struct type.
  // inline void filter_cloud(dv_msgs::PointCloud scan) {
  //
  // }
}
