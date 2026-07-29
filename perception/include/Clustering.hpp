
#pragma once

#include <cstddef>
#include <vector>

#include "PointCloudTypes.hpp"
namespace perception
{
struct ClusteringParams
{
    double cluster_tolerance_m{0.25};
    std::size_t min_cluster_size{3};
    std::size_t max_cluster_size{1000};
};

struct Cluster
{
    std::vector<std::size_t> point_indices;
};

struct ClusteringDebug
{
    std::size_t input_points{};
    std::size_t raw_cluster_count{};
    std::size_t accepted_cluster_count{};
    std::size_t rejected_too_small{};
    std::size_t rejected_too_large{};
};

struct ClusteringResult
{
    std::vector<Cluster> clusters;
    std::int64_t timestamp_ns;
    ClusteringDebug debug;
};

ClusteringResult euclidean_cluster_xy(
    const StampedPointCloud& non_ground_points, const ClusteringParams& params);

}  // namespace perception
