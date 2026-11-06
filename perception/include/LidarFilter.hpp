#pragma once

#include <algorithm>

#include "StateTracker.hpp"
#include "dv_msgs.pb.h"

namespace perception {

  // Lidar filtering constants
  constexpr float GRID_MIN_THETA_RAD = -1.5707963f;
  constexpr float GRID_MAX_THETA_RAD = 1.5707963f;
  constexpr float GRID_ANGULAR_BIN_SIZE_RAD = 0.08726646f;
  constexpr float GRID_MIN_RANGE_M = 1.0f;
  constexpr float GRID_MAX_RANGE_M = 26.0f;
  constexpr float GRID_RADIAL_BIN_SIZE_M = 0.75f;

  // Ground estimation. The only absolute height in the filter: where the ground sits in sensor z
  // when a cell has too few points to estimate it (the lidar is ~0.6 m above the ground).
  constexpr float SENSOR_HEIGHT_ABOVE_GROUND_M = 0.6f;
  constexpr std::size_t MIN_POINTS_PER_CELL = 5;
  constexpr float GROUND_PERCENTILE = 0.15f;
  constexpr float NON_GROUND_HEIGHT_THRESHOLD_M = 0.05f;

  // Clustering
  constexpr float CLUSTER_TOLERANCE_M = 0.25f;
  constexpr int MIN_CLUSTER_SIZE = 3;
  constexpr int MAX_CLUSTER_SIZE = 1000;

  // Cone Filtering
  constexpr float MAX_DETECTION_RANGE_M = 20.0f;
  constexpr float NEAR_RANGE_M = 5.0f;
  constexpr float MID_RANGE_M = 15.0f;

  constexpr int NEAR_MIN_CONE_POINTS = 7;
  constexpr int MID_MIN_CONE_POINTS = 3;
  constexpr int FAR_MIN_CONE_POINTS = 4;

  constexpr float NEAR_MIN_CONE_HEIGHT_M = 0.10f;
  constexpr float MID_MIN_CONE_HEIGHT_M = 0.05f;
  constexpr float FAR_MIN_CONE_HEIGHT_M = 0.08f;

  constexpr float NEAR_MAX_CONE_WIDTH_M = 0.30f;
  constexpr float MID_MAX_CONE_WIDTH_M = 0.15f;
  constexpr float FAR_MAX_CONE_WIDTH_M = 0.12f;

  constexpr float MAX_CONE_HEIGHT_M = 0.35f;
  // Cluster top above the local ground; catches tall objects whose base the ground filter removed
  constexpr float MAX_CONE_TOP_HEIGHT_M = 0.45f;
  constexpr float MAX_ELONGATION = 4.0f;
  constexpr float MIN_WIDTH_FOR_ELONGATION_M = 0.03f;

  // Detection confidence: 1.0 out to NEAR_RANGE_M, then linear down to MIN_CONFIDENCE at MAX_DETECTION_RANGE_M
  constexpr float MIN_CONFIDENCE = 0.2f;

  inline float detection_confidence(float range_m) {
    const float s = (range_m - NEAR_RANGE_M) / (MAX_DETECTION_RANGE_M - NEAR_RANGE_M);
    return std::clamp(1.0f - (1.0f - MIN_CONFIDENCE) * s, MIN_CONFIDENCE, 1.0f);
  }

  /**
   * Performs ground and object filtering on a LiDAR scan
   * @param scan The pointcloud to run filtering on
   * @return shared pointer to the filtered cones
  */
  std::shared_ptr<dv_msgs::Cones> filter_cloud(const foxglove::PointCloud& scan);
} // namespace perception


