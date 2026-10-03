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

  // Cone Filtering
  constexpr float MAX_DETECTION_RANGE_M = 25.0f;
  constexpr float NEAR_RANGE_M = 5.0f;
  constexpr float MID_RANGE_M = 10.0f;

  constexpr int NEAR_MIN_CONE_POINTS = 7;
  constexpr int MID_MIN_CONE_POINTS = 5;
  constexpr int FAR_MIN_CONE_POINTS = 3;

  constexpr float NEAR_MIN_CONE_HEIGHT_M = 0.06f;
  constexpr float MID_MIN_CONE_HEIGHT_M = 0.02f;
  constexpr float FAR_MIN_CONE_HEIGHT_M = 0.0f;

  constexpr float NEAR_MAX_CONE_WIDTH_M = 0.30f;
  constexpr float MID_MAX_CONE_WIDTH_M = 0.25f;
  constexpr float FAR_MAX_CONE_WIDTH_M = 0.10f;

  constexpr float MAX_CONE_HEIGHT_M = 0.5f;
  constexpr float MAX_ELONGATION = 4.0f;
  constexpr float MIN_WIDTH_FOR_ELONGATION_M = 0.03f;

  constexpr float NEAR_ACCEPTED_CONFIDENCE = 1.0f;
  constexpr float MID_ACCEPTED_CONFIDENCE = 0.5f;
  constexpr float FAR_ACCEPTED_CONFIDENCE = 0.25f;

  /**
   * Performs ground and object filtering on a LiDAR scan
   * @param scan The pointcloud to run filtering on
   * @return shared pointer to the filtered cones
  */
  std::shared_ptr<dv_msgs::Cones> filter_cloud(const foxglove::PointCloud& scan);
} // namespace perception


