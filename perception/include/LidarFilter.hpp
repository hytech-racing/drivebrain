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

  // Ground estimation. Himmelsbach-style piecewise linear regression.
  // Selects the lowest point in each radial bin and fits local ground segments.
  // Ensures validity by checking for continuity and slope as well as fitting error.
  constexpr float SENSOR_HEIGHT_ABOVE_GROUND_M = 0.6f;
  constexpr std::size_t MIN_POINTS_PER_CELL = 5;
  constexpr float NON_GROUND_HEIGHT_THRESHOLD_M = 0.05f;
  constexpr std::size_t MIN_GROUND_LINE_POINTS = 3;
  constexpr float MAX_GROUND_SLOPE = 0.25f;
  constexpr float MAX_GROUND_FIT_ERROR_M = 0.05f;
  constexpr float MAX_GROUND_SEGMENT_GAP_M = 1.5f;
  constexpr float MAX_GROUND_START_HEIGHT_ERROR_M = 0.20f;
  constexpr float MAX_GROUND_CONTINUITY_ERROR_M = 0.15f;
  constexpr float MAX_GROUND_POINT_LINE_DISTANCE_M = 0.08f;
  constexpr float GROUND_LONG_GAP_M = 1.0f;
  constexpr float MAX_GROUND_LONG_GAP_HEIGHT_CHANGE_M = 0.20f;
  constexpr int GROUND_NEIGHBOR_SECTOR_SEARCH_BINS = 1;



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


