
#pragma once

#include <cstddef>
#include <vector>

#include "Clustering.hpp"
#include "PointCloudTypes.hpp"

namespace perception
{

struct BoundingBox
{
    PointXYZI min;
    PointXYZI max;
};

struct ClusterFeatures
{
    PointXYZI centroid;
    BoundingBox bbox;

    std::size_t num_points{};

    double width_x_m{};
    double width_y_m{};
    double height_z_m{};

    double max_horizontal_width_m{};

    double range_m{};
};

ClusterFeatures compute_cluster_features(
    const StampedPointCloud& non_ground_points, const Cluster& cluster);

std::vector<ClusterFeatures> compute_all_cluster_features(
    const StampedPointCloud& non_ground_points,
    const std::vector<Cluster>& clusters);

}  // namespace perception
