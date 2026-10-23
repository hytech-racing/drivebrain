#include "PointCloudDeskew.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

#include <Geometry.hpp>
#include <StateTracker.hpp>
#include <foxglove/PackedElementField.pb.h>

namespace perception {
namespace {

  struct Layout {
    int stride = 0;
    int x = -1, y = -1, z = -1, t = -1;  // byte offsets
  };

  std::optional<Layout> float_layout(const foxglove::PointCloud& scan) {
    Layout layout;
    layout.stride = static_cast<int>(scan.point_stride());
    for (const auto& field : scan.fields()) {
      if (field.type() != foxglove::PackedElementField::FLOAT32) continue;
      const int offset = static_cast<int>(field.offset());
      if (field.name() == "x") layout.x = offset;
      else if (field.name() == "y") layout.y = offset;
      else if (field.name() == "z") layout.z = offset;
      else if (field.name() == "t") layout.t = offset;
    }
    if (layout.stride <= 0 || layout.x < 0 || layout.y < 0 || layout.z < 0) return std::nullopt;
    return layout;
  }

}  // namespace

std::shared_ptr<const foxglove::PointCloud> deskew_cloud(const foxglove::PointCloud& scan,
                                                         const Eigen::Vector3d& twist_base,
                                                         const Eigen::Isometry3d& T_base_lidar) {
  const auto layout = float_layout(scan);
  if (!layout) return std::make_shared<foxglove::PointCloud>(scan);

  const std::size_t n = scan.data().size() / layout->stride;
  const char* in = scan.data().data();

  // xyz as-is, which is also the fallback when the scan has no per-point times
  std::vector<core::LidarPoint> points(n);
  for (std::size_t i = 0; i < n; i++) {
    const char* p = in + i * layout->stride;
    points[i] = {*reinterpret_cast<const float*>(p + layout->x), *reinterpret_cast<const float*>(p + layout->y),
                 *reinterpret_cast<const float*>(p + layout->z)};
  }
  if (layout->t < 0 || n == 0) return core::to_cloud(points, scan);

  // Time span of the scan relative to its timestamp
  float t_min = 0.0f, t_max = 0.0f;
  for (std::size_t i = 0; i < n; i++) {
    const float t = *reinterpret_cast<const float*>(in + i * layout->stride + layout->t);
    t_min = std::min(t_min, t);
    t_max = std::max(t_max, t);
  }
  const float t_span = t_max - t_min;
  if (!(t_span > 0.0f)) return core::to_cloud(points, scan);

  // Lidar pose of each time bin relative to the lidar pose at the scan timestamp
  const Eigen::Isometry3d T_lidar_base = T_base_lidar.inverse();
  std::vector<Eigen::Isometry3d> T_ref_from_bin(DESKEW_TIME_BINS);
  for (int b = 0; b < DESKEW_TIME_BINS; b++) {
    const double dt = t_min + (b + 0.5) * t_span / DESKEW_TIME_BINS;
    // SE(2) exponential of the constant twist over dt
    const double theta = twist_base.z() * dt;
    const Eigen::Vector2d v = twist_base.head<2>() * dt;
    const bool straight = std::abs(theta) < 1e-6;
    const double s = straight ? 1.0 : std::sin(theta) / theta;
    const double c = straight ? 0.0 : (1.0 - std::cos(theta)) / theta;
    const Eigen::Isometry2d T_base_ref_from_base_bin =
        core::geometry::make_pose2d(s * v.x() - c * v.y(), c * v.x() + s * v.y(), theta);
    T_ref_from_bin[b] = T_lidar_base * core::geometry::to_pose3d(T_base_ref_from_base_bin) * T_base_lidar;
  }

  for (std::size_t i = 0; i < n; i++) {
    const float t = *reinterpret_cast<const float*>(in + i * layout->stride + layout->t);
    const int b = std::clamp(static_cast<int>((t - t_min) / t_span * DESKEW_TIME_BINS), 0, DESKEW_TIME_BINS - 1);
    const Eigen::Vector3d p = T_ref_from_bin[b] * Eigen::Vector3d(points[i].x, points[i].y, points[i].z);
    points[i] = {static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z())};
  }
  return core::to_cloud(points, scan);
}

}  // namespace perception
