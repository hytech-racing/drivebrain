#include "PointCloudDeskew.hpp"

#include <vector>

#include <Geometry.hpp>
#include <StateTracker.hpp>

namespace perception {

std::shared_ptr<const foxglove::PointCloud> deskew_cloud(const foxglove::PointCloud& scan,
                                                         const Eigen::Vector3d& twist_base,
                                                         const Eigen::Isometry3d& T_base_lidar) {
  int t_offset = -1;
  for (const auto& field : scan.fields()) {
    if (field.name() == "t") t_offset = static_cast<int>(field.offset());
  }

  const std::size_t stride = scan.point_stride();
  const std::size_t n = stride ? scan.data().size() / stride : 0;
  const char* in = scan.data().data();
  const Eigen::Isometry3d T_lidar_base = T_base_lidar.inverse();

  std::vector<core::LidarPoint> points(n);
  for (std::size_t i = 0; i < n; i++) {
    const char* p = in + i * stride;
    points[i] = *reinterpret_cast<const core::LidarPoint*>(p);
    if (t_offset < 0) continue;

    // Where the car is at the scan timestamp, seen from where it was when this point was measured
    const float t = *reinterpret_cast<const float*>(p + t_offset);
    const Eigen::Isometry2d motion = core::geometry::integrate_twist(twist_base, t);

    Eigen::Vector3d q = T_base_lidar * Eigen::Vector3d(points[i].x, points[i].y, points[i].z);
    q.head<2>() = motion * q.head<2>();
    q = T_lidar_base * q;
    points[i] = {static_cast<float>(q.x()), static_cast<float>(q.y()), static_cast<float>(q.z())};
  }
  return core::to_cloud(points, scan);
}

}  // namespace perception
