#pragma once

#include <Eigen/Geometry>

#include "dv_msgs.pb.h"

namespace perception {

  // A LiDAR cone takes the color of a true sim cone at most this far away
  constexpr double SIM_COLOR_MATCH_M = 0.6;

  /**
   * HOOTL only: the sim has no cameras, so no cone ever gets a color. Copies the color of the nearest true cone
   * instead. Does nothing on the real car (no ground truth is ever set). Colors already set by the camera classifier
   * are kept.
   * @param cones The cones from the LiDAR filter, os_sensor frame
   * @param T_base_lidar The lidar mounting pose in base_link (the sim's car reference point)
  */
  void color_cones_from_sim_ground_truth(dv_msgs::Cones& cones, const Eigen::Isometry3d& T_base_lidar);

}
