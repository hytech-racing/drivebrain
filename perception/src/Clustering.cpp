
#include "Clustering.hpp"

#include <queue>
#include <vector>

namespace perception
{

ClusteringResult euclidean_cluster_xy(const StampedPointCloud& input,
                                      const ClusteringParams& params)
{
    ClusteringResult result;
    result.timestamp_ns = input.timestamp_ns;

    const PointCloud& non_ground_points = input.points;

    std::vector<bool> visited =
        std::vector<bool>(non_ground_points.size(), false);

    std::size_t raw_cluster_count = 0;
    std::size_t rejected_too_large = 0;
    std::size_t rejected_too_small = 0;

    const double tolerance_squared =
        params.cluster_tolerance_m * params.cluster_tolerance_m;

    for (std::size_t main_point_index = 0;
         main_point_index < non_ground_points.size(); ++main_point_index)
    {
        if (visited[main_point_index])
        {
            continue;
        }

        Cluster cluster;
        raw_cluster_count++;
        std::queue<std::size_t> queue;
        queue.push(main_point_index);
        visited[main_point_index] = true;

        while (!queue.empty())
        {
            const std::size_t current = queue.front();
            cluster.point_indices.push_back(current);

            queue.pop();

            for (std::size_t comparison_point_index = 0;
                 comparison_point_index < non_ground_points.size();
                 ++comparison_point_index)
            {
                if (visited[comparison_point_index])
                {
                    continue;
                }

                const double dx = non_ground_points[current].x -
                                  non_ground_points[comparison_point_index].x;
                const double dy = non_ground_points[current].y -
                                  non_ground_points[comparison_point_index].y;

                const double distance_squared = dx * dx + dy * dy;

                if (distance_squared <= tolerance_squared)
                {
                    visited[comparison_point_index] = true;
                    queue.push(comparison_point_index);
                }
            }
        }

        if (cluster.point_indices.size() < params.min_cluster_size)
        {
            rejected_too_small++;
            continue;
        }

        if (cluster.point_indices.size() > params.max_cluster_size)
        {
            rejected_too_large++;
            continue;
        }

        result.clusters.push_back(cluster);
    }

    result.debug.input_points = non_ground_points.size();
    result.debug.raw_cluster_count = raw_cluster_count;
    result.debug.accepted_cluster_count = result.clusters.size();
    result.debug.rejected_too_large = rejected_too_large;
    result.debug.rejected_too_small = rejected_too_small;

    return result;
}

}  // namespace perception
