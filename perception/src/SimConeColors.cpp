#include "SimConeColors.hpp"

#include <cmath>

#include "StateTracker.hpp"

namespace perception {

  void color_cones_from_sim_ground_truth(dv_msgs::Cones& cones, const Eigen::Isometry3d& T_base_lidar) {
    const auto dv = core::StateTracker::instance().dv_state();
    if (!dv.ground_truth_cones || !dv.has_ground_truth_pose) return;

    // os_sensor -> base_link -> sim world frame
    const Eigen::Isometry3d T_world_base = Eigen::Translation3d(dv.ground_truth_x, dv.ground_truth_y, 0.0) *
                                           Eigen::AngleAxisd(dv.ground_truth_yaw, Eigen::Vector3d::UnitZ());
    const Eigen::Isometry3d T_world_lidar = T_world_base * T_base_lidar;

    for (auto& cone : *cones.mutable_cones()) {
      if (cone.color() != dv_msgs::Cones_ConeColor_UNKNOWN) continue;
      const Eigen::Vector3d w = T_world_lidar * Eigen::Vector3d(cone.position().x(), cone.position().y(), cone.position().z());
      double best = SIM_COLOR_MATCH_M;
      for (const auto& truth : dv.ground_truth_cones->cones()) {
        const double d = std::hypot(truth.position().x() - w.x(), truth.position().y() - w.y());
        if (d < best) {
          best = d;
          cone.set_color(truth.color());
          cone.set_color_confidence(1.0f);  // SLAM's colour voting ignores zero-confidence colours
        }
      }
    }
  }

}
