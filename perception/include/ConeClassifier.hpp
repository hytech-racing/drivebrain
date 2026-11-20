#pragma once

#include <array>
#include <map>
#include <string>

#include <Eigen/Geometry>

#include "StateTracker.hpp"
#include "dv_msgs.pb.h"

namespace perception {

  constexpr float CONE_HEIGHT_M = 0.23495f;
  constexpr float CONE_WIDTH_M = 0.127f;

  // the mr cal model
  struct CameraModel {
    float fx, fy, cx, cy;
    std::array<float, 8> dist;
    int width, height;

    // Pixel to the undistorted normalized coords (x/z, y/z) of its ray
    Eigen::Vector2f unproject(const Eigen::Vector2f& px) const;

    // Camera-frame point to distorted image pixels (requires positive depth)
    Eigen::Vector2f project(const Eigen::Vector3f& point) const;
  };

  CameraModel load_camera_model(const std::string& path);

  class ConeClassifier {
    public:

      ConeClassifier(const std::string& intrinsics_dir);

      /**
       * Colors cones by projecting LiDAR centroids into each camera and matching to containing YOLO boxes,
       * closest to the box center first. Each detection colors at most one cone per camera;
       * unmatched cones stay UNKNOWN.
       * @param cones The uncolored cones from the LiDAR filter, os_sensor frame
       * @param detections The latest detections per camera, keyed by camera name
       * @param frame_id The LiDAR scan's frame, for drawing the camera-placed cones next to the LiDAR ones
       * @return The cones with colors assigned
      */
      std::shared_ptr<dv_msgs::Cones> classify(std::shared_ptr<dv_msgs::Cones> cones,
                                               const std::map<std::string, std::shared_ptr<const std::vector<core::ConeDetection>>>& detections,
                                               const std::string& frame_id);

    private:
      std::map<std::string, CameraModel> _cameras;
  };

}