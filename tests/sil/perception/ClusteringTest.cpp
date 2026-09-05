#include <gtest/gtest.h>

#include <Clustering.hpp>
#include <PointCloudTypes.hpp>

#include <algorithm>
#include <cstddef>
#include <utility>
#include <vector>

namespace perception
{
namespace
{

PointXYZI make_point(const float x, const float y, const float z = 0.0F,
                     const float intensity = 1.0F)
{
    PointXYZI point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.intensity = intensity;
    return point;
}

StampedPointCloud make_cloud(PointCloud points)
{
    StampedPointCloud cloud;
    cloud.timestamp_ns = 100;
    cloud.frame = transforms::FrameId::Lidar;
    cloud.points = std::move(points);
    return cloud;
}

std::vector<std::size_t> sorted_indices(const Cluster& cluster)
{
    std::vector<std::size_t> indices = cluster.point_indices;
    std::sort(indices.begin(), indices.end());
    return indices;
}

ClusteringParams make_default_params()
{
    ClusteringParams params;
    params.cluster_tolerance_m = 0.25;
    params.min_cluster_size = 2;
    params.max_cluster_size = 100;
    return params;
}

TEST(ClusteringTest, EmptyInputReturnsNoClusters)
{
    const PointCloud points;
    const ClusteringParams params = make_default_params();

    const ClusteringResult result = euclidean_cluster_xy(make_cloud(points), params);

    EXPECT_TRUE(result.clusters.empty());
    EXPECT_EQ(result.timestamp_ns, 100);
    EXPECT_EQ(result.debug.input_points, 0U);
    EXPECT_EQ(result.debug.raw_cluster_count, 0U);
    EXPECT_EQ(result.debug.accepted_cluster_count, 0U);
    EXPECT_EQ(result.debug.rejected_too_small, 0U);
    EXPECT_EQ(result.debug.rejected_too_large, 0U);
}

TEST(ClusteringTest, RejectsSinglePointWhenBelowMinimumClusterSize)
{
    PointCloud points;
    points.push_back(make_point(0.0F, 0.0F));

    ClusteringParams params = make_default_params();
    params.min_cluster_size = 2;

    const ClusteringResult result = euclidean_cluster_xy(make_cloud(points), params);

    EXPECT_TRUE(result.clusters.empty());
    EXPECT_EQ(result.debug.input_points, 1U);
    EXPECT_EQ(result.debug.raw_cluster_count, 1U);
    EXPECT_EQ(result.debug.accepted_cluster_count, 0U);
    EXPECT_EQ(result.debug.rejected_too_small, 1U);
    EXPECT_EQ(result.debug.rejected_too_large, 0U);
}

TEST(ClusteringTest, GroupsNearbyPointsIntoOneCluster)
{
    PointCloud points;
    points.push_back(make_point(0.00F, 0.00F));
    points.push_back(make_point(0.10F, 0.00F));
    points.push_back(make_point(0.18F, 0.05F));

    const ClusteringParams params = make_default_params();

    const ClusteringResult result = euclidean_cluster_xy(make_cloud(points), params);

    ASSERT_EQ(result.clusters.size(), 1U);
    EXPECT_EQ(result.debug.raw_cluster_count, 1U);
    EXPECT_EQ(result.debug.accepted_cluster_count, 1U);

    const std::vector<std::size_t> expected_indices{0U, 1U, 2U};
    EXPECT_EQ(sorted_indices(result.clusters.at(0)), expected_indices);
}

TEST(ClusteringTest, SeparatesTwoDistantGroups)
{
    PointCloud points;

    points.push_back(make_point(0.00F, 0.00F));
    points.push_back(make_point(0.10F, 0.00F));
    points.push_back(make_point(0.18F, 0.05F));

    points.push_back(make_point(2.00F, 0.00F));
    points.push_back(make_point(2.10F, 0.02F));

    const ClusteringParams params = make_default_params();

    const ClusteringResult result = euclidean_cluster_xy(make_cloud(points), params);

    ASSERT_EQ(result.clusters.size(), 2U);
    EXPECT_EQ(result.debug.raw_cluster_count, 2U);
    EXPECT_EQ(result.debug.accepted_cluster_count, 2U);

    EXPECT_EQ(sorted_indices(result.clusters.at(0)),
              (std::vector<std::size_t>{0U, 1U, 2U}));
    EXPECT_EQ(sorted_indices(result.clusters.at(1)),
              (std::vector<std::size_t>{3U, 4U}));
}

TEST(ClusteringTest, ClusteringDoesNotDependOnPointVectorOrder)
{
    PointCloud points;

    points.push_back(make_point(0.00F, 0.00F));
    points.push_back(make_point(8.00F, 0.00F));
    points.push_back(make_point(2.00F, 0.00F));
    points.push_back(make_point(0.10F, 0.02F));
    points.push_back(make_point(2.08F, 0.04F));
    points.push_back(make_point(0.18F, 0.04F));

    ClusteringParams params = make_default_params();
    params.cluster_tolerance_m = 0.25;
    params.min_cluster_size = 2;

    const ClusteringResult result = euclidean_cluster_xy(make_cloud(points), params);

    ASSERT_EQ(result.clusters.size(), 2U);
    EXPECT_EQ(result.debug.raw_cluster_count, 3U);
    EXPECT_EQ(result.debug.accepted_cluster_count, 2U);
    EXPECT_EQ(result.debug.rejected_too_small, 1U);

    EXPECT_EQ(sorted_indices(result.clusters.at(0)),
              (std::vector<std::size_t>{0U, 3U, 5U}));
    EXPECT_EQ(sorted_indices(result.clusters.at(1)),
              (std::vector<std::size_t>{2U, 4U}));
}

TEST(ClusteringTest, UsesConnectedChainBehavior)
{
    PointCloud points;

    points.push_back(make_point(0.00F, 0.00F));
    points.push_back(make_point(0.19F, 0.00F));
    points.push_back(make_point(0.38F, 0.00F));

    ClusteringParams params = make_default_params();
    params.cluster_tolerance_m = 0.20;
    params.min_cluster_size = 1;

    const ClusteringResult result = euclidean_cluster_xy(make_cloud(points), params);

    ASSERT_EQ(result.clusters.size(), 1U);
    EXPECT_EQ(sorted_indices(result.clusters.at(0)),
              (std::vector<std::size_t>{0U, 1U, 2U}));
}

TEST(ClusteringTest, UsesOnlyXYDistanceAndIgnoresZ)
{
    PointCloud points;

    points.push_back(make_point(1.00F, 0.00F, 0.00F));
    points.push_back(make_point(1.05F, 0.02F, 0.40F));

    ClusteringParams params = make_default_params();
    params.cluster_tolerance_m = 0.10;
    params.min_cluster_size = 2;

    const ClusteringResult result = euclidean_cluster_xy(make_cloud(points), params);

    ASSERT_EQ(result.clusters.size(), 1U);
    EXPECT_EQ(sorted_indices(result.clusters.at(0)),
              (std::vector<std::size_t>{0U, 1U}));
}

TEST(ClusteringTest, RejectsClusterLargerThanMaximumSize)
{
    PointCloud points;

    points.push_back(make_point(0.00F, 0.00F));
    points.push_back(make_point(0.05F, 0.00F));
    points.push_back(make_point(0.10F, 0.00F));
    points.push_back(make_point(0.15F, 0.00F));

    ClusteringParams params = make_default_params();
    params.cluster_tolerance_m = 0.10;
    params.min_cluster_size = 1;
    params.max_cluster_size = 3;

    const ClusteringResult result = euclidean_cluster_xy(make_cloud(points), params);

    EXPECT_TRUE(result.clusters.empty());
    EXPECT_EQ(result.debug.raw_cluster_count, 1U);
    EXPECT_EQ(result.debug.accepted_cluster_count, 0U);
    EXPECT_EQ(result.debug.rejected_too_large, 1U);
    EXPECT_EQ(result.debug.rejected_too_small, 0U);
}

TEST(ClusteringTest, BoundaryDistanceIsIncluded)
{
    PointCloud points;

    points.push_back(make_point(0.00F, 0.00F));
    points.push_back(make_point(0.25F, 0.00F));

    ClusteringParams params = make_default_params();
    params.cluster_tolerance_m = 0.25;
    params.min_cluster_size = 2;

    const ClusteringResult result = euclidean_cluster_xy(make_cloud(points), params);

    ASSERT_EQ(result.clusters.size(), 1U);
    EXPECT_EQ(sorted_indices(result.clusters.at(0)),
              (std::vector<std::size_t>{0U, 1U}));
}

}  // namespace
}  // namespace perception
