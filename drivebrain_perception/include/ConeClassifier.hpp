#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>

#include <Eigen/Geometry>

#include "StateTracker.hpp"
#include "dv_msgs.pb.h"

namespace perception {

  constexpr float CONE_HEIGHT_M = 0.325f;
  // A detection matches a cone when its center is within this many box heights of the projected cone center
  constexpr float MATCH_GATE = 0.5f;
  // and its height is within this factor of the height expected at the LiDAR range
  constexpr float MAX_SIZE_RATIO = 2.0f;
  constexpr float MIN_PROJECTION_DEPTH_M = 0.5f;

  // the mr cal model
  struct CameraModel {
    float fx, fy, cx, cy;
    std::array<float, 8> dist;
    int width, height;

    std::optional<Eigen::Vector2f> project(const Eigen::Vector3f& p_cam) const;
  };

  CameraModel load_camera_model(const std::string& path);

  class ConeClassifier {
    public:

      ConeClassifier(const std::string& intrinsics_dir);

      /**
       * Colors cones by projecting them into each camera and matching them to that camera's YOLO detections.
       * Each detection colors at most one cone; unmatched cones stay UNKNOWN.
       * @param cones The uncolored cones from the LiDAR filter, os_sensor frame
       * @param detections The latest detections per camera, keyed by camera name
       * @return The cones with colors assigned
      */
      std::shared_ptr<dv_msgs::Cones> classify(std::shared_ptr<dv_msgs::Cones> cones,
                                               const std::map<std::string, std::shared_ptr<const std::vector<core::ConeDetection>>>& detections);

    private:
      std::map<std::string, CameraModel> _cameras; 
  };

}
