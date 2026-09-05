#include <gtest/gtest.h>

#include <PointCloudDeskew.hpp>

namespace perception
{
namespace
{

TEST(PointCloudDeskewTest, EmitsOnePointPerInputPoint)
{
    StampedPointCloud cloud;
    cloud.timestamp_ns = 200;
    cloud.points = {{1.0F, 0.0F, 0.0F, 1.0F},
                    {2.0F, 0.0F, 0.0F, 2.0F},
                    {3.0F, 0.0F, 0.0F, 3.0F}};

    StampedLidarPose reference;
    reference.stamp_ns = 200;
    reference.pose = transforms::Pose3D{};

    std::vector<StampedLidarPose> poses;
    poses.resize(cloud.points.size());
    for (std::size_t i = 0; i < poses.size(); ++i)
    {
        poses[i].stamp_ns = static_cast<std::int64_t>(i + 1);
        poses[i].pose = transforms::Pose3D{};
    }

    const DeskewResult result = deskew_point_cloud(cloud, reference, poses);

    ASSERT_EQ(result.stamped_point_cloud.points.size(), cloud.points.size());
    EXPECT_EQ(result.stamped_point_cloud.timestamp_ns, reference.stamp_ns);
    EXPECT_EQ(result.start_stamp_ns, poses.front().stamp_ns);
    EXPECT_EQ(result.end_stamp_ns, cloud.timestamp_ns);
    EXPECT_EQ(result.reference_stamp_ns, reference.stamp_ns);
}

TEST(PointCloudDeskewTest, AppliesMatchingPoseToEachPoint)
{
    StampedPointCloud cloud;
    cloud.timestamp_ns = 200;
    cloud.points = {{1.0F, 0.0F, 0.0F, 1.0F}, {2.0F, 0.0F, 0.0F, 2.0F}};

    StampedLidarPose reference;
    reference.stamp_ns = 200;
    reference.pose = transforms::Pose3D{};

    std::vector<StampedLidarPose> poses = {
        StampedLidarPose{100, transforms::Pose3D{10.0, 0.0, 0.0, {}}},
        StampedLidarPose{200, transforms::Pose3D{20.0, 0.0, 0.0, {}}}};

    const DeskewResult result = deskew_point_cloud(cloud, reference, poses);

    ASSERT_EQ(result.stamped_point_cloud.points.size(), 2U);
    EXPECT_FLOAT_EQ(result.stamped_point_cloud.points[0].x, 11.0F);
    EXPECT_FLOAT_EQ(result.stamped_point_cloud.points[1].x, 22.0F);
    EXPECT_FLOAT_EQ(result.stamped_point_cloud.points[0].intensity, 1.0F);
    EXPECT_FLOAT_EQ(result.stamped_point_cloud.points[1].intensity, 2.0F);
}

TEST(PointCloudDeskewTest, EmptyCloudReturnsEmptyResult)
{
    StampedPointCloud cloud;
    cloud.timestamp_ns = 200;

    StampedLidarPose reference;
    reference.stamp_ns = 200;

    const DeskewResult result = deskew_point_cloud(cloud, reference, {});

    EXPECT_TRUE(result.stamped_point_cloud.points.empty());
    EXPECT_EQ(result.stamped_point_cloud.timestamp_ns, reference.stamp_ns);
    EXPECT_EQ(result.start_stamp_ns, reference.stamp_ns);
    EXPECT_EQ(result.end_stamp_ns, cloud.timestamp_ns);
    EXPECT_EQ(result.reference_stamp_ns, reference.stamp_ns);
}

}  // namespace
}  // namespace perception
