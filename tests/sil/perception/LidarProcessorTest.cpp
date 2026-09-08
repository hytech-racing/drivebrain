#include <gtest/gtest.h>

#include <LidarProcessor.hpp>

#include <cstdint>
#include <optional>
#include <utility>

namespace perception
{
namespace
{

constexpr std::int64_t kTimestampNs = 123456789;

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

StampedPointCloud make_cloud(PointCloud points,
                             const transforms::FrameId frame =
                                 transforms::FrameId::Lidar)
{
    StampedPointCloud cloud;
    cloud.timestamp_ns = kTimestampNs;
    cloud.frame = frame;
    cloud.points = std::move(points);
    return cloud;
}

void expect_cloud_metadata(const StampedPointCloud& cloud)
{
    EXPECT_EQ(cloud.timestamp_ns, kTimestampNs);
    EXPECT_EQ(cloud.frame, transforms::FrameId::Lidar);
}

TEST(LidarProcessorTest, ProcessesSyntheticCloudEndToEnd)
{
    PointCloud points;

    for (int i = 0; i < 5; ++i)
    {
        points.push_back(make_point(2.0F, static_cast<float>(i) * 0.001F,
                                    0.0F));
    }

    for (int i = 0; i < 10; ++i)
    {
        const float x = 1.95F + static_cast<float>(i % 5) * 0.025F;
        const float y = -0.05F + static_cast<float>(i / 5) * 0.10F;
        const float z = (i % 2 == 0) ? 0.20F : 0.40F;
        points.push_back(make_point(x, y, z));
    }

    for (int i = 0; i < 10; ++i)
    {
        const float x = 3.80F + static_cast<float>(i) * 0.05F;
        const float z = (i % 2 == 0) ? 0.20F : 0.40F;
        points.push_back(make_point(x, 1.0F, z));
    }

    LidarProcessorParams params;
    params.ground_removal_params.grid_min_range_m = 0.0;
    params.ground_removal_params.grid_max_range_m = 10.0;
    params.ground_removal_params.radial_bin_size_m = 10.0;
    params.ground_removal_params.grid_min_theta_rad = -1.6;
    params.ground_removal_params.grid_max_theta_rad = 1.6;
    params.ground_removal_params.angular_bin_size_rad = 3.2;

    LidarProcessor processor{params};

    const std::optional<LidarProcessingResult> result =
        processor.process(LidarProcessingInput{make_cloud(points)});

    ASSERT_TRUE(result.has_value());

    expect_cloud_metadata(result->deskewed_point_cloud);
    expect_cloud_metadata(result->filtered_point_cloud);
    expect_cloud_metadata(result->ground_point_cloud);
    expect_cloud_metadata(result->non_ground_point_cloud);

    EXPECT_EQ(result->ground_point_cloud.points.size(), 5U);
    ASSERT_EQ(result->clusters.size(), 2U);
    ASSERT_EQ(result->cluster_features.size(), 2U);

    ASSERT_EQ(result->cone_candidates.size(), 1U);
    EXPECT_NEAR(result->cone_candidates.at(0).position.x, 2.0, 0.1);
    EXPECT_EQ(result->rejected_clusters.size(), 1U);
    EXPECT_EQ(result->rejected_clusters.at(0).reason,
              ConeRejectionReason::TooWide);
}

TEST(LidarProcessorTest, RejectsNonLidarFrame)
{
    PointCloud points;
    points.push_back(make_point(2.0F, 0.0F, 0.1F));

    LidarProcessor processor{LidarProcessorParams{}};

    const std::optional<LidarProcessingResult> result = processor.process(
        LidarProcessingInput{make_cloud(points, transforms::FrameId::Baselink)});

    EXPECT_FALSE(result.has_value());
}

TEST(LidarProcessorTest, EmptyLidarCloudProducesValidEmptyResult)
{
    LidarProcessor processor{LidarProcessorParams{}};

    const std::optional<LidarProcessingResult> result =
        processor.process(LidarProcessingInput{make_cloud({})});

    ASSERT_TRUE(result.has_value());

    expect_cloud_metadata(result->deskewed_point_cloud);
    expect_cloud_metadata(result->filtered_point_cloud);
    expect_cloud_metadata(result->ground_point_cloud);
    expect_cloud_metadata(result->non_ground_point_cloud);

    EXPECT_TRUE(result->ground_point_cloud.points.empty());
    EXPECT_TRUE(result->non_ground_point_cloud.points.empty());
    EXPECT_TRUE(result->clusters.empty());
    EXPECT_TRUE(result->cluster_features.empty());
    EXPECT_TRUE(result->cone_candidates.empty());
    EXPECT_TRUE(result->rejected_clusters.empty());
}

TEST(LidarProcessorTest, DeskewDisabledWithoutMotionSucceeds)
{
    PointCloud points;
    points.push_back(make_point(2.0F, 0.0F, 0.1F));

    LidarProcessorParams params;
    params.deskew_enabled = false;
    LidarProcessor processor{params};

    const std::optional<LidarProcessingResult> result =
        processor.process(LidarProcessingInput{make_cloud(points)});

    EXPECT_TRUE(result.has_value());
}

TEST(LidarProcessorTest, DeskewEnabledWithoutMotionRejectsInput)
{
    PointCloud points;
    points.push_back(make_point(2.0F, 0.0F, 0.1F));

    LidarProcessorParams params;
    params.deskew_enabled = true;
    LidarProcessor processor{params};

    const std::optional<LidarProcessingResult> result =
        processor.process(LidarProcessingInput{make_cloud(points)});

    EXPECT_FALSE(result.has_value());
}

}  // namespace
}  // namespace perception
