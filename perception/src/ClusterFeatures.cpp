
#include "ClusterFeatures.hpp"

#include <cmath>
#include <cstddef>

namespace perception
{

ClusterFeatures compute_cluster_features(const StampedPointCloud& input,
                                         const Cluster& cluster)
{
    ClusterFeatures cluster_features;

    const PointCloud& non_ground_points = input.points;

    if (cluster.point_indices.empty())
    {
        return cluster_features;
    }

    cluster_features.bbox.min.x =
        non_ground_points.at(cluster.point_indices.front()).x;
    cluster_features.bbox.min.y =
        non_ground_points.at(cluster.point_indices.front()).y;
    cluster_features.bbox.min.z =
        non_ground_points.at(cluster.point_indices.front()).z;

    cluster_features.bbox.max.x = cluster_features.bbox.min.x;
    cluster_features.bbox.max.y = cluster_features.bbox.min.y;
    cluster_features.bbox.max.z = cluster_features.bbox.min.z;

    const std::size_t num_points_in_cluster = cluster.point_indices.size();

    for (std::size_t point_index : cluster.point_indices)
    {
        const PointXYZI& curr_point = non_ground_points.at(point_index);

        cluster_features.centroid.x += curr_point.x;
        cluster_features.centroid.y += curr_point.y;
        cluster_features.centroid.z += curr_point.z;

        if (cluster_features.bbox.min.x > curr_point.x)
        {
            cluster_features.bbox.min.x = curr_point.x;
        }

        if (cluster_features.bbox.min.y > curr_point.y)
        {
            cluster_features.bbox.min.y = curr_point.y;
        }

        if (cluster_features.bbox.min.z > curr_point.z)
        {
            cluster_features.bbox.min.z = curr_point.z;
        }

        if (cluster_features.bbox.max.x < curr_point.x)
        {
            cluster_features.bbox.max.x = curr_point.x;
        }

        if (cluster_features.bbox.max.y < curr_point.y)
        {
            cluster_features.bbox.max.y = curr_point.y;
        }

        if (cluster_features.bbox.max.z < curr_point.z)
        {
            cluster_features.bbox.max.z = curr_point.z;
        }
    }

    cluster_features.centroid.x /= num_points_in_cluster;
    cluster_features.centroid.y /= num_points_in_cluster;
    cluster_features.centroid.z /= num_points_in_cluster;

    cluster_features.width_x_m =
        cluster_features.bbox.max.x - cluster_features.bbox.min.x;
    cluster_features.width_y_m =
        cluster_features.bbox.max.y - cluster_features.bbox.min.y;
    cluster_features.height_z_m =
        cluster_features.bbox.max.z - cluster_features.bbox.min.z;

    cluster_features.max_horizontal_width_m =
        std::max(cluster_features.width_x_m, cluster_features.width_y_m);
    cluster_features.range_m =
        std::sqrt((cluster_features.centroid.x * cluster_features.centroid.x) +
                  (cluster_features.centroid.y * cluster_features.centroid.y));

    cluster_features.num_points = num_points_in_cluster;

    return cluster_features;
}

std::vector<ClusterFeatures> compute_all_cluster_features(
    const StampedPointCloud& non_ground_points,
    const std::vector<Cluster>& clusters)
{
    std::vector<ClusterFeatures> cluster_features;

    if (clusters.empty())
    {
        return cluster_features;
    }

    cluster_features.reserve(clusters.size());

    for (const Cluster& cluster : clusters)
    {
        cluster_features.push_back(
            compute_cluster_features(non_ground_points, cluster));
    }

    return cluster_features;
}

}  // namespace perception
