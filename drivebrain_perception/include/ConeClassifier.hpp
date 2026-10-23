#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>

#include <Eigen/Geometry>
#include <foxglove/RawImage.pb.h>

#include "StateTracker.hpp"
#include "dv_msgs.pb.h"
#if JETSON_ENABLED
#include "Detector.hpp"
#endif

namespace perception {

  constexpr float CONE_HEIGHT_M = 0.325f;
  // Side of the square patch around each cone, in metres, so patches frame cones the same at any range
  constexpr float PATCH_SIZE_M = 0.4f;
  // Side of the patches the color model takes, in pixels
  constexpr int PATCH_PX = 32;
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
       * Projects each cone into every camera with a frame and logs the boxes to <camera>/cone_boxes.
       * On the Jetson, colors each cone from its most confident camera patch.
       * @param cones The uncolored cones from the LiDAR filter, os_sensor frame
       * @param frames The latest frame per camera, keyed by frame_id
       * @return The cones
      */
      std::shared_ptr<dv_msgs::Cones> classify(std::shared_ptr<dv_msgs::Cones> cones, const std::map<std::string, std::shared_ptr<const foxglove::RawImage>>& frames);

    private:
      std::map<std::string, CameraModel> _cameras; 
#if JETSON_ENABLED
      std::unique_ptr<comms::Detector> _detector;
#endif
  };

}
