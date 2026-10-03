#pragma once

#include "StateTracker.hpp"
#include "dv_msgs.pb.h"

namespace perception {

  // Lidar filtering constants
  constexpr float GRID_MIN_THETA_RAD = -1.5707963f;
  constexpr float GRID_MAX_THETA_RAD = 1.5707963f;
  constexpr float GRID_ANGULAR_BIN_SIZE_RAD = 0.08726646f;
  constexpr float GRID_MIN_RANGE_M = 1.0f;
  constexpr float GRID_MAX_RANGE_M = 35.0f;
  constexpr float GRID_RADIAL_BIN_SIZE_M = 0.5f;

  // Ground estimation
  constexpr std::size_t MIN_POINTS_PER_CELL = 5;
  constexpr float FLAT_GROUND_Z_MAX_M = -0.5f;
  constexpr float GROUND_PERCENTILE = 0.15f;
  constexpr float NON_GROUND_HEIGHT_THRESHOLD_M = 0.02f;

  // Clustering
  constexpr float CLUSTER_TOLERANCE_M = 0.25f;
  constexpr int MIN_CLUSTER_SIZE = 3;
  constexpr int MAX_CLUSTER_SIZE = 1000;

  /**
   * Performs ground and object filtering on a LiDAR scan
   * @param scan The pointcloud to run filtering on
   * @return goon
  */
  std::shared_ptr<const foxglove::PointCloud> filter_cloud(const foxglove::PointCloud& scan);
}


