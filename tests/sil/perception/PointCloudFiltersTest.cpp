#include <gtest/gtest.h>

#include <PointCloudFilters.hpp>

#include <cmath>
#include <limits>
#include <utility>

namespace perception
{
namespace
{

constexpr double kPi = 3.14159265358979323846264338327950288;

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

TEST(PointCloudFiltersTest, RemoveNonFiniteRemovesNanAndInf)
{
    PointCloud input;
    input.push_back(make_point(1.0F, 0.0F, 0.0F));
    input.push_back(
        make_point(std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F));
    input.push_back(
        make_point(0.0F, std::numeric_limits<float>::infinity(), 0.0F));
    input.push_back(
        make_point(0.0F, 0.0F, -std::numeric_limits<float>::infinity()));
    input.push_back(make_point(2.0F, 0.0F, 0.0F));

    const StampedPointCloud output = remove_non_finite(make_cloud(input));

    ASSERT_EQ(output.points.size(), 2U);
    EXPECT_EQ(output.timestamp_ns, 100);
    EXPECT_EQ(output.frame, transforms::FrameId::Lidar);
    EXPECT_FLOAT_EQ(output.points.at(0).x, 1.0F);
    EXPECT_FLOAT_EQ(output.points.at(1).x, 2.0F);
}

TEST(PointCloudFiltersTest, CropByAngleKeepsFrontHalfPlane)
{
    const AngleCropConfig config{
        -kPi / 2.0,
        kPi / 2.0,
    };

    PointCloud input;
    input.push_back(make_point(1.0F, 0.0F, 0.0F));
    input.push_back(make_point(1.0F, 1.0F, 0.0F));
    input.push_back(make_point(1.0F, -1.0F, 0.0F));
    input.push_back(make_point(0.1F, 1.0F, 0.0F));
    input.push_back(make_point(0.1F, -1.0F, 0.0F));
    input.push_back(make_point(-1.0F, 0.0F, 0.0F));
    input.push_back(make_point(-1.0F, 1.0F, 0.0F));
    input.push_back(make_point(-1.0F, -1.0F, 0.0F));

    const StampedPointCloud output = crop_by_angle(make_cloud(input), config);

    ASSERT_EQ(output.points.size(), 5U);
}

TEST(PointCloudFiltersTest, CropByAngleCanSelectNarrowForwardCone)
{
    const AngleCropConfig config{
        -kPi / 4.0,
        kPi / 4.0,
    };

    PointCloud input;
    input.push_back(make_point(1.0F, 0.0F, 0.0F));
    input.push_back(make_point(1.0F, 0.99F, 0.0F));
    input.push_back(make_point(1.0F, -0.99F, 0.0F));
    input.push_back(make_point(1.0F, 1.01F, 0.0F));
    input.push_back(make_point(1.0F, -1.01F, 0.0F));
    input.push_back(make_point(0.0F, 1.0F, 0.0F));
    input.push_back(make_point(0.0F, -1.0F, 0.0F));

    const StampedPointCloud output = crop_by_angle(make_cloud(input), config);

    ASSERT_EQ(output.points.size(), 3U);
}

TEST(PointCloudFiltersTest, CropByRoiKeepsOnlyPointsInsideBox)
{
    const RoiConfig config{
        0.0,
        20.0,
        -8.0,
        8.0,
        -0.5,
        1.0,
    };

    PointCloud input;
    input.push_back(make_point(1.0F, 0.0F, 0.0F));
    input.push_back(make_point(20.0F, 8.0F, 1.0F));
    input.push_back(make_point(0.0F, -8.0F, -0.5F));
    input.push_back(make_point(-0.1F, 0.0F, 0.0F));
    input.push_back(make_point(20.1F, 0.0F, 0.0F));
    input.push_back(make_point(1.0F, -8.1F, 0.0F));
    input.push_back(make_point(1.0F, 8.1F, 0.0F));
    input.push_back(make_point(1.0F, 0.0F, -0.6F));
    input.push_back(make_point(1.0F, 0.0F, 1.1F));

    const StampedPointCloud output = crop_by_roi(make_cloud(input), config);

    ASSERT_EQ(output.points.size(), 3U);
}

TEST(PointCloudFiltersTest, FullPreprocessingOrderWorksForSyntheticCloud)
{
    PointCloudFilterParams params;
    params.angle_crop_enabled = true;
    params.roi_crop_enabled = true;
    params.angle_crop = {-kPi / 2.0, kPi / 2.0};
    params.roi = {0.0, 20.0, -8.0, 8.0, -0.5, 1.0};

    PointCloud input;
    input.push_back(make_point(1.0F, 0.0F, 0.0F));
    input.push_back(make_point(2.0F, 1.0F, 0.2F));
    input.push_back(make_point(-1.0F, 0.0F, 0.0F));
    input.push_back(make_point(1.0F, 9.0F, 0.0F));
    input.push_back(make_point(1.0F, 0.0F, 2.0F));
    input.push_back(make_point(std::numeric_limits<float>::quiet_NaN(), 0.0F,
                               0.0F));

    const StampedPointCloud output = filter_point_cloud(make_cloud(input), params);

    ASSERT_EQ(output.points.size(), 2U);

    EXPECT_FLOAT_EQ(output.points.at(0).x, 1.0F);
    EXPECT_FLOAT_EQ(output.points.at(0).y, 0.0F);
    EXPECT_FLOAT_EQ(output.points.at(0).z, 0.0F);

    EXPECT_FLOAT_EQ(output.points.at(1).x, 2.0F);
    EXPECT_FLOAT_EQ(output.points.at(1).y, 1.0F);
    EXPECT_FLOAT_EQ(output.points.at(1).z, 0.2F);
}

}  // namespace
}  // namespace perception
