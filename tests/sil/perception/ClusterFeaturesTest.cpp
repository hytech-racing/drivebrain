#include <gtest/gtest.h>

#include <ClusterFeatures.hpp>
#include <Clustering.hpp>
#include <PointCloudTypes.hpp>

#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace perception
{
namespace
{

PointXYZI make_point(const float x, const float y, const float z,
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

Cluster make_cluster(const std::vector<std::size_t>& indices)
{
    Cluster cluster;
    cluster.point_indices = indices;
    return cluster;
}

TEST(ClusterFeaturesTest, EmptyClusterReturnsDefaultFeatures)
{
    PointCloud points;
    points.push_back(make_point(1.0F, 2.0F, 3.0F));

    const Cluster empty_cluster;

    const ClusterFeatures features =
        compute_cluster_features(make_cloud(points), empty_cluster);

    EXPECT_EQ(features.num_points, 0U);

    EXPECT_FLOAT_EQ(features.centroid.x, 0.0F);
    EXPECT_FLOAT_EQ(features.centroid.y, 0.0F);
    EXPECT_FLOAT_EQ(features.centroid.z, 0.0F);

    EXPECT_FLOAT_EQ(features.bbox.min.x, 0.0F);
    EXPECT_FLOAT_EQ(features.bbox.min.y, 0.0F);
    EXPECT_FLOAT_EQ(features.bbox.min.z, 0.0F);

    EXPECT_FLOAT_EQ(features.bbox.max.x, 0.0F);
    EXPECT_FLOAT_EQ(features.bbox.max.y, 0.0F);
    EXPECT_FLOAT_EQ(features.bbox.max.z, 0.0F);

    EXPECT_DOUBLE_EQ(features.width_x_m, 0.0);
    EXPECT_DOUBLE_EQ(features.width_y_m, 0.0);
    EXPECT_DOUBLE_EQ(features.height_z_m, 0.0);
    EXPECT_DOUBLE_EQ(features.max_horizontal_width_m, 0.0);
    EXPECT_DOUBLE_EQ(features.range_m, 0.0);
}

TEST(ClusterFeaturesTest, ComputesCentroidBoundingBoxWidthsHeightAndRange)
{
    PointCloud points;

    points.push_back(make_point(1.0F, 2.0F, 0.1F));
    points.push_back(make_point(2.0F, 4.0F, 0.4F));
    points.push_back(make_point(4.0F, 3.0F, 0.7F));

    const Cluster cluster = make_cluster({0U, 1U, 2U});

    const ClusterFeatures features =
        compute_cluster_features(make_cloud(points), cluster);

    EXPECT_EQ(features.num_points, 3U);

    EXPECT_NEAR(features.centroid.x, (1.0 + 2.0 + 4.0) / 3.0, 1e-6);
    EXPECT_NEAR(features.centroid.y, (2.0 + 4.0 + 3.0) / 3.0, 1e-6);
    EXPECT_NEAR(features.centroid.z, (0.1 + 0.4 + 0.7) / 3.0, 1e-6);

    EXPECT_FLOAT_EQ(features.bbox.min.x, 1.0F);
    EXPECT_FLOAT_EQ(features.bbox.min.y, 2.0F);
    EXPECT_FLOAT_EQ(features.bbox.min.z, 0.1F);

    EXPECT_FLOAT_EQ(features.bbox.max.x, 4.0F);
    EXPECT_FLOAT_EQ(features.bbox.max.y, 4.0F);
    EXPECT_FLOAT_EQ(features.bbox.max.z, 0.7F);

    EXPECT_NEAR(features.width_x_m, 3.0, 1e-6);
    EXPECT_NEAR(features.width_y_m, 2.0, 1e-6);
    EXPECT_NEAR(features.height_z_m, 0.6, 1e-6);

    EXPECT_NEAR(features.max_horizontal_width_m, 3.0, 1e-6);

    const double expected_range =
        std::sqrt(features.centroid.x * features.centroid.x +
                  features.centroid.y * features.centroid.y);

    EXPECT_NEAR(features.range_m, expected_range, 1e-6);
}

TEST(ClusterFeaturesTest, UsesOnlyPointsListedInClusterIndices)
{
    PointCloud points;

    points.push_back(make_point(100.0F, 100.0F, 100.0F));
    points.push_back(make_point(1.0F, 2.0F, 0.1F));
    points.push_back(make_point(2.0F, 4.0F, 0.4F));
    points.push_back(make_point(-50.0F, -50.0F, -50.0F));

    const Cluster cluster = make_cluster({1U, 2U});

    const ClusterFeatures features =
        compute_cluster_features(make_cloud(points), cluster);

    EXPECT_EQ(features.num_points, 2U);

    EXPECT_NEAR(features.centroid.x, 1.5, 1e-6);
    EXPECT_NEAR(features.centroid.y, 3.0, 1e-6);
    EXPECT_NEAR(features.centroid.z, 0.25, 1e-6);

    EXPECT_FLOAT_EQ(features.bbox.min.x, 1.0F);
    EXPECT_FLOAT_EQ(features.bbox.min.y, 2.0F);
    EXPECT_FLOAT_EQ(features.bbox.min.z, 0.1F);

    EXPECT_FLOAT_EQ(features.bbox.max.x, 2.0F);
    EXPECT_FLOAT_EQ(features.bbox.max.y, 4.0F);
    EXPECT_FLOAT_EQ(features.bbox.max.z, 0.4F);

    EXPECT_NEAR(features.width_x_m, 1.0, 1e-6);
    EXPECT_NEAR(features.width_y_m, 2.0, 1e-6);
    EXPECT_NEAR(features.height_z_m, 0.3, 1e-6);
    EXPECT_NEAR(features.max_horizontal_width_m, 2.0, 1e-6);
}

TEST(ClusterFeaturesTest, HandlesSingletonCluster)
{
    PointCloud points;
    points.push_back(make_point(3.0F, 4.0F, 0.5F));

    const Cluster cluster = make_cluster({0U});

    const ClusterFeatures features =
        compute_cluster_features(make_cloud(points), cluster);

    EXPECT_EQ(features.num_points, 1U);

    EXPECT_FLOAT_EQ(features.centroid.x, 3.0F);
    EXPECT_FLOAT_EQ(features.centroid.y, 4.0F);
    EXPECT_FLOAT_EQ(features.centroid.z, 0.5F);

    EXPECT_FLOAT_EQ(features.bbox.min.x, 3.0F);
    EXPECT_FLOAT_EQ(features.bbox.min.y, 4.0F);
    EXPECT_FLOAT_EQ(features.bbox.min.z, 0.5F);

    EXPECT_FLOAT_EQ(features.bbox.max.x, 3.0F);
    EXPECT_FLOAT_EQ(features.bbox.max.y, 4.0F);
    EXPECT_FLOAT_EQ(features.bbox.max.z, 0.5F);

    EXPECT_DOUBLE_EQ(features.width_x_m, 0.0);
    EXPECT_DOUBLE_EQ(features.width_y_m, 0.0);
    EXPECT_DOUBLE_EQ(features.height_z_m, 0.0);
    EXPECT_DOUBLE_EQ(features.max_horizontal_width_m, 0.0);

    EXPECT_NEAR(features.range_m, 5.0, 1e-6);
}

TEST(ClusterFeaturesTest, ComputeAllClusterFeaturesReturnsOneFeaturePerCluster)
{
    PointCloud points;

    points.push_back(make_point(0.0F, 0.0F, 0.1F));
    points.push_back(make_point(0.2F, 0.0F, 0.3F));

    points.push_back(make_point(2.0F, 1.0F, 0.2F));
    points.push_back(make_point(2.4F, 1.2F, 0.6F));
    points.push_back(make_point(2.2F, 1.4F, 0.4F));

    std::vector<Cluster> clusters;
    clusters.push_back(make_cluster({0U, 1U}));
    clusters.push_back(make_cluster({2U, 3U, 4U}));

    const std::vector<ClusterFeatures> all_features =
        compute_all_cluster_features(make_cloud(points), clusters);

    ASSERT_EQ(all_features.size(), 2U);

    EXPECT_EQ(all_features.at(0).num_points, 2U);
    EXPECT_NEAR(all_features.at(0).centroid.x, 0.1, 1e-6);
    EXPECT_NEAR(all_features.at(0).centroid.y, 0.0, 1e-6);
    EXPECT_NEAR(all_features.at(0).centroid.z, 0.2, 1e-6);
    EXPECT_NEAR(all_features.at(0).width_x_m, 0.2, 1e-6);
    EXPECT_NEAR(all_features.at(0).width_y_m, 0.0, 1e-6);
    EXPECT_NEAR(all_features.at(0).height_z_m, 0.2, 1e-6);

    EXPECT_EQ(all_features.at(1).num_points, 3U);
    EXPECT_NEAR(all_features.at(1).centroid.x, (2.0 + 2.4 + 2.2) / 3.0, 1e-6);
    EXPECT_NEAR(all_features.at(1).centroid.y, (1.0 + 1.2 + 1.4) / 3.0, 1e-6);
    EXPECT_NEAR(all_features.at(1).centroid.z, (0.2 + 0.6 + 0.4) / 3.0, 1e-6);
    EXPECT_NEAR(all_features.at(1).width_x_m, 0.4, 1e-6);
    EXPECT_NEAR(all_features.at(1).width_y_m, 0.4, 1e-6);
    EXPECT_NEAR(all_features.at(1).height_z_m, 0.4, 1e-6);
}

TEST(ClusterFeaturesTest, ComputeAllClusterFeaturesWithNoClustersReturnsEmpty)
{
    PointCloud points;
    points.push_back(make_point(1.0F, 2.0F, 3.0F));

    const std::vector<Cluster> clusters;

    const std::vector<ClusterFeatures> all_features =
        compute_all_cluster_features(make_cloud(points), clusters);

    EXPECT_TRUE(all_features.empty());
}

TEST(ClusterFeaturesTest, InvalidClusterIndexThrowsOutOfRange)
{
    PointCloud points;
    points.push_back(make_point(1.0F, 2.0F, 3.0F));

    const Cluster cluster = make_cluster({0U, 5U});

    EXPECT_THROW(
        {
            const ClusterFeatures features =
                compute_cluster_features(make_cloud(points), cluster);
            (void)features;
        },
        std::out_of_range);
}

}  // namespace
}  // namespace perception
