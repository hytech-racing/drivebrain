#pragma once

#include <memory>

#include <Eigen/Geometry>
#include <foxglove/PointCloud.pb.h>

namespace perception {

  /**
   * Moves every point of a scan to where it would have been measured from the scan's timestamp,
   * assuming the car moved at a constant twist over the scan. Each point's time comes from the
   * cloud's `t` field (seconds relative to the cloud timestamp).
   *
   * @param scan the scan: x/y/z float32 at offsets 0/4/8, optionally a `t` float32 field
   * @param twist_base body velocity (vx m/s, vy m/s, yaw rate rad/s) in base_link
   * @param T_base_lidar the lidar mounting pose in base_link
   * @return an xyz-only cloud (stride 12, as the lidar filter expects) at the scan timestamp; a plain
   *         xyz copy when the scan has no per-point times
   */
  std::shared_ptr<const foxglove::PointCloud> deskew_cloud(const foxglove::PointCloud& scan,
                                                           const Eigen::Vector3d& twist_base,
                                                           const Eigen::Isometry3d& T_base_lidar);

}  // namespace perception
