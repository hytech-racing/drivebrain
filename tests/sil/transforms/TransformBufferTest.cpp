#include <gtest/gtest.h>

#include <FrameId.hpp>
#include <RigidTransform2D.hpp>
#include <RigidTransform3D.hpp>
#include <TransformBuffer.hpp>
#include <cmath>
#include <chrono>
#include <limits>
#include <optional>
#include <thread>

namespace transforms
{
namespace
{

constexpr double kTolerance = 1e-9;
constexpr double kPi = 3.14159265358979323846;

using namespace std::chrono_literals;

void expect_transform_near(const Pose2D& actual, const Pose2D& expected)
{
    EXPECT_NEAR(actual.x_m, expected.x_m, kTolerance);
    EXPECT_NEAR(actual.y_m, expected.y_m, kTolerance);
    EXPECT_NEAR(actual.yaw_rad, expected.yaw_rad, kTolerance);
}

void expect_pose3d_near(const Pose3D& actual, const Pose3D& expected)
{
    EXPECT_NEAR(actual.x_m, expected.x_m, kTolerance);
    EXPECT_NEAR(actual.y_m, expected.y_m, kTolerance);
    EXPECT_NEAR(actual.z_m, expected.z_m, kTolerance);
    EXPECT_NEAR(actual.q.w, expected.q.w, kTolerance);
    EXPECT_NEAR(actual.q.x, expected.q.x, kTolerance);
    EXPECT_NEAR(actual.q.y, expected.q.y, kTolerance);
    EXPECT_NEAR(actual.q.z, expected.q.z, kTolerance);
}

Quaternion yaw_quaternion(const double yaw_rad)
{
    return Quaternion{std::cos(0.5 * yaw_rad), 0.0, 0.0,
                      std::sin(0.5 * yaw_rad)};
}

Quaternion pitch_quaternion(const double pitch_rad)
{
    return Quaternion{std::cos(0.5 * pitch_rad), 0.0,
                      std::sin(0.5 * pitch_rad), 0.0};
}

TEST(TransformBufferTest, ExactTimestampLookup)
{
    TransformBuffer buffer(1000);
    const Pose2D expected{1.0, 2.0, 0.3};

    EXPECT_TRUE(buffer.insert_T_odom_base(100, expected));
    EXPECT_TRUE(buffer.insert_T_odom_base(200, {3.0, 4.0, 0.6}));

    const std::optional<Pose2D> actual =
        buffer.lookup(FrameId::Odom, FrameId::Baselink, 100);

    ASSERT_TRUE(actual.has_value());
    expect_transform_near(*actual, expected);
}

TEST(TransformBufferTest, MidpointTranslationInterpolation)
{
    TransformBuffer buffer(1000);

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {0.0, 2.0, 0.0}));
    EXPECT_TRUE(buffer.insert_T_odom_base(200, {10.0, 6.0, 0.0}));

    const std::optional<Pose2D> actual =
        buffer.lookup(FrameId::Odom, FrameId::Baselink, 150);

    ASSERT_TRUE(actual.has_value());
    expect_transform_near(*actual, {5.0, 4.0, 0.0});
}

TEST(TransformBufferTest, YawInterpolationAcrossPiBoundary)
{
    TransformBuffer buffer(1000);

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {0.0, 0.0, kPi - 0.1}));
    EXPECT_TRUE(buffer.insert_T_odom_base(200, {0.0, 0.0, -kPi + 0.1}));

    const std::optional<Pose2D> actual =
        buffer.lookup(FrameId::Odom, FrameId::Baselink, 150);

    ASSERT_TRUE(actual.has_value());
    EXPECT_NEAR(actual->yaw_rad, -kPi, kTolerance);
}

TEST(TransformBufferTest, RequestOlderThanHistoryReturnsNullopt)
{
    TransformBuffer buffer(1000);

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {1.0, 2.0, 0.3}));
    EXPECT_TRUE(buffer.insert_T_odom_base(200, {3.0, 4.0, 0.6}));

    EXPECT_EQ(buffer.lookup(FrameId::Odom, FrameId::Baselink, 50),
              std::nullopt);
}

TEST(TransformBufferTest, RequestNewerThanNewestReturnsNullopt)
{
    TransformBuffer buffer(1000);

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {1.0, 2.0, 0.3}));
    EXPECT_TRUE(buffer.insert_T_odom_base(200, {3.0, 4.0, 0.6}));

    EXPECT_EQ(buffer.lookup(FrameId::Odom, FrameId::Baselink, 250),
              std::nullopt);
}

TEST(TransformBufferTest, ZeroTimeoutKeepsImmediateLookupBehavior)
{
    TransformBuffer buffer(1000);

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {1.0, 2.0, 0.3}));

    EXPECT_EQ(buffer.lookup(FrameId::Odom, FrameId::Baselink, 200, 0ns),
              std::nullopt);
}

TEST(TransformBufferTest, LookupWaitsForFutureOdomSample)
{
    TransformBuffer buffer(1000);

    std::thread inserter(
        [&buffer]()
        {
            std::this_thread::sleep_for(1ms);
            EXPECT_TRUE(buffer.insert_T_odom_base(200, {1.0, 2.0, 0.3}));
        });

    const std::optional<Pose2D> actual =
        buffer.lookup(FrameId::Odom, FrameId::Baselink, 200, 50ms);

    inserter.join();

    ASSERT_TRUE(actual.has_value());
    expect_transform_near(*actual, {1.0, 2.0, 0.3});
}

TEST(TransformBufferTest, LookupReturnsNulloptAfterTimeout)
{
    TransformBuffer buffer(1000);

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {1.0, 2.0, 0.3}));

    EXPECT_EQ(buffer.lookup(FrameId::Odom, FrameId::Baselink, 200, 1ms),
              std::nullopt);
}

TEST(TransformBufferTest, HistoryPruningBySensorTimestamp)
{
    TransformBuffer buffer(100);

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {1.0, 0.0, 0.0}));
    EXPECT_TRUE(buffer.insert_T_odom_base(150, {2.0, 0.0, 0.0}));
    EXPECT_TRUE(buffer.insert_T_odom_base(225, {3.0, 0.0, 0.0}));

    EXPECT_EQ(buffer.lookup(FrameId::Odom, FrameId::Baselink, 100),
              std::nullopt);

    const std::optional<Pose2D> retained =
        buffer.lookup(FrameId::Odom, FrameId::Baselink, 150);
    ASSERT_TRUE(retained.has_value());
    expect_transform_near(*retained, {2.0, 0.0, 0.0});
}

TEST(TransformBufferTest, OutOfOrderInsertionRejected)
{
    TransformBuffer buffer(1000);
    const Pose2D original_latest{2.0, 0.0, 0.0};

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {1.0, 0.0, 0.0}));
    EXPECT_TRUE(buffer.insert_T_odom_base(200, original_latest));
    EXPECT_FALSE(buffer.insert_T_odom_base(150, {9.0, 0.0, 0.0}));

    const std::optional<Pose2D> latest =
        buffer.lookup(FrameId::Odom, FrameId::Baselink, 200);
    ASSERT_TRUE(latest.has_value());
    expect_transform_near(*latest, original_latest);
}

TEST(TransformBufferTest, SameTimestampInsertionReplacesExistingSample)
{
    TransformBuffer buffer(1000);
    const Pose2D replacement{2.0, 3.0, 0.4};

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {1.0, 0.0, 0.0}));
    EXPECT_TRUE(buffer.insert_T_odom_base(100, replacement));

    const std::optional<Pose2D> actual =
        buffer.lookup(FrameId::Odom, FrameId::Baselink, 100);

    ASSERT_TRUE(actual.has_value());
    expect_transform_near(*actual, replacement);
}

TEST(TransformBufferTest, InvalidDynamicInsertionRejected)
{
    TransformBuffer buffer(1000);
    constexpr double nan = std::numeric_limits<double>::quiet_NaN();

    EXPECT_FALSE(buffer.insert_T_odom_base(0, {1.0, 2.0, 0.3}));
    EXPECT_FALSE(buffer.insert_T_odom_base(100, {nan, 2.0, 0.3}));
    EXPECT_FALSE(buffer.insert_T_odom_base(100, {1.0, nan, 0.3}));
    EXPECT_FALSE(buffer.insert_T_odom_base(100, {1.0, 2.0, nan}));
    EXPECT_FALSE(buffer.insert_T_map_odom(0, {1.0, 2.0, 0.3}));
    EXPECT_FALSE(buffer.insert_T_map_odom(100, {nan, 2.0, 0.3}));
    EXPECT_FALSE(buffer.insert_T_map_odom(100, {1.0, nan, 0.3}));
    EXPECT_FALSE(buffer.insert_T_map_odom(100, {1.0, 2.0, nan}));
    EXPECT_FALSE(buffer.insert_T_odom_base3d(
        100, Pose3D{1.0, 2.0, nan, Quaternion{1.0, 0.0, 0.0, 0.0}}));
    EXPECT_FALSE(buffer.insert_T_odom_base3d(
        100, Pose3D{1.0, 2.0, 3.0, Quaternion{0.0, 0.0, 0.0, 0.0}}));

    EXPECT_EQ(buffer.lookup(FrameId::Odom, FrameId::Baselink, 100),
              std::nullopt);
    EXPECT_EQ(buffer.lookup(FrameId::Map, FrameId::Odom, 100), std::nullopt);
}

TEST(TransformBufferTest, MapOdomLookupHoldsLatestSampleAtOrBeforeQuery)
{
    TransformBuffer buffer(1000);
    const Pose2D before{1.0, 2.0, 0.3};
    const Pose2D after{10.0, 20.0, 1.3};

    EXPECT_TRUE(buffer.insert_T_map_odom(100, before));
    EXPECT_TRUE(buffer.insert_T_map_odom(200, after));

    const std::optional<Pose2D> actual =
        buffer.lookup(FrameId::Map, FrameId::Odom, 150);

    ASSERT_TRUE(actual.has_value());
    expect_transform_near(*actual, before);
}

TEST(TransformBufferTest, ZeroDurationRetainsNewestSample)
{
    TransformBuffer buffer(0);

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {1.0, 0.0, 0.0}));
    EXPECT_TRUE(buffer.insert_T_odom_base(200, {2.0, 0.0, 0.0}));

    EXPECT_EQ(buffer.lookup(FrameId::Odom, FrameId::Baselink, 100),
              std::nullopt);

    const auto latest = buffer.lookup(FrameId::Odom, FrameId::Baselink, 200);

    ASSERT_TRUE(latest.has_value());
}

TEST(TransformBufferTest, Lookup3dConvertsLookup2dResult)
{
    TransformBuffer buffer(1000);

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {1.0, 2.0, kPi / 2.0}));

    const std::optional<Pose3D> actual =
        buffer.lookup3d(FrameId::Odom, FrameId::Baselink, 100);

    ASSERT_TRUE(actual.has_value());
    EXPECT_NEAR(actual->x_m, 1.0, kTolerance);
    EXPECT_NEAR(actual->y_m, 2.0, kTolerance);
    EXPECT_NEAR(actual->z_m, 0.0, kTolerance);
    EXPECT_NEAR(actual->q.w, std::cos(kPi / 4.0), kTolerance);
    EXPECT_NEAR(actual->q.x, 0.0, kTolerance);
    EXPECT_NEAR(actual->q.y, 0.0, kTolerance);
    EXPECT_NEAR(actual->q.z, std::sin(kPi / 4.0), kTolerance);
}

TEST(TransformBufferTest, Lookup3dPreservesNative3dDynamicPose)
{
    TransformBuffer buffer(1000);
    const Pose3D expected{1.0, 2.0, 3.0, pitch_quaternion(0.4)};

    EXPECT_TRUE(buffer.insert_T_odom_base3d(100, expected));

    const std::optional<Pose3D> actual =
        buffer.lookup3d(FrameId::Odom, FrameId::Baselink, 100);

    ASSERT_TRUE(actual.has_value());
    expect_pose3d_near(*actual, expected);
}

TEST(TransformBufferTest, LookupProjectsNative3dDynamicPoseTo2d)
{
    TransformBuffer buffer(1000);
    EXPECT_TRUE(buffer.insert_T_odom_base3d(
        100, Pose3D{1.0, 2.0, 3.0, pitch_quaternion(0.4)}));

    const std::optional<Pose2D> actual =
        buffer.lookup(FrameId::Odom, FrameId::Baselink, 100);

    ASSERT_TRUE(actual.has_value());
    expect_transform_near(*actual, Pose2D{1.0, 2.0, 0.0});
}

TEST(TransformBufferTest, Lookup3dInterpolatesTranslationAndQuaternion)
{
    TransformBuffer buffer(1000);

    EXPECT_TRUE(buffer.insert_T_odom_base3d(
        100, Pose3D{0.0, 0.0, 0.0, yaw_quaternion(0.0)}));
    EXPECT_TRUE(buffer.insert_T_odom_base3d(
        200, Pose3D{10.0, 20.0, 4.0, yaw_quaternion(kPi)}));

    const std::optional<Pose3D> actual =
        buffer.lookup3d(FrameId::Odom, FrameId::Baselink, 150);

    ASSERT_TRUE(actual.has_value());
    EXPECT_NEAR(actual->x_m, 5.0, kTolerance);
    EXPECT_NEAR(actual->y_m, 10.0, kTolerance);
    EXPECT_NEAR(actual->z_m, 2.0, kTolerance);
    expect_transform_near(actual->to_pose2d(), Pose2D{5.0, 10.0, kPi / 2.0});
}

TEST(TransformBufferTest, StaticSensor3dTransformParticipatesInLookup3d)
{
    TransformBuffer buffer(1000);
    const Pose3D T_base_lidar{1.0, 2.0, 0.5, pitch_quaternion(0.2)};

    EXPECT_TRUE(buffer.set_T_base_lidar3d(T_base_lidar));
    EXPECT_TRUE(buffer.insert_T_odom_base(100, Pose2D{}));

    const std::optional<Pose3D> actual =
        buffer.lookup3d(FrameId::Baselink, FrameId::Lidar, 100);

    ASSERT_TRUE(actual.has_value());
    expect_pose3d_near(*actual, T_base_lidar);
}

TEST(TransformBufferTest, StaticSensor3dTransformProjectsForLookup)
{
    TransformBuffer buffer(1000);

    EXPECT_TRUE(buffer.set_T_base_lidar3d(
        Pose3D{1.0, 2.0, 0.5, pitch_quaternion(0.2)}));
    EXPECT_TRUE(buffer.insert_T_odom_base(100, Pose2D{}));

    const std::optional<Pose2D> actual =
        buffer.lookup(FrameId::Baselink, FrameId::Lidar, 100);

    ASSERT_TRUE(actual.has_value());
    expect_transform_near(*actual, Pose2D{1.0, 2.0, 0.0});
}

TEST(TransformBufferTest, CameraStaticTransformsParticipateInLookup3d)
{
    TransformBuffer buffer(1000);
    const Pose3D expected{0.0, 0.0, 1.0, Quaternion{}};

    EXPECT_TRUE(buffer.set_T_base_camera_wide3d(expected));
    EXPECT_TRUE(buffer.set_T_base_camera_narrow3d(expected));
    EXPECT_TRUE(buffer.insert_T_odom_base(100, Pose2D{}));

    const std::optional<Pose3D> wide =
        buffer.lookup3d(FrameId::Baselink, FrameId::CameraWide, 100);
    const std::optional<Pose3D> narrow =
        buffer.lookup3d(FrameId::Baselink, FrameId::CameraNarrow, 100);

    ASSERT_TRUE(wide.has_value());
    ASSERT_TRUE(narrow.has_value());
    expect_pose3d_near(*wide, expected);
    expect_pose3d_near(*narrow, expected);
}

TEST(TransformBufferTest, CameraToLidarLookupComposesThroughBaseLink)
{
    TransformBuffer buffer(1000);

    EXPECT_TRUE(buffer.set_T_base_camera_wide3d(
        Pose3D{0.0, 0.0, 1.0, Quaternion{}}));
    EXPECT_TRUE(buffer.set_T_base_lidar3d(
        Pose3D{0.75, 0.0, 0.15, Quaternion{}}));
    EXPECT_TRUE(buffer.insert_T_odom_base(100, Pose2D{}));

    const std::optional<Pose3D> T_camera_lidar =
        buffer.lookup3d(FrameId::CameraWide, FrameId::Lidar, 100);

    ASSERT_TRUE(T_camera_lidar.has_value());
    expect_pose3d_near(*T_camera_lidar,
                       Pose3D{0.75, 0.0, -0.85, Quaternion{}});
}

TEST(TransformBufferTest, Lookup3dWaitsForFutureOdomSample)
{
    TransformBuffer buffer(1000);

    std::thread inserter(
        [&buffer]()
        {
            std::this_thread::sleep_for(1ms);
            EXPECT_TRUE(buffer.insert_T_odom_base(100, {1.0, 2.0, kPi / 2.0}));
        });

    const std::optional<Pose3D> actual =
        buffer.lookup3d(FrameId::Odom, FrameId::Baselink, 100, 50ms);

    inserter.join();

    ASSERT_TRUE(actual.has_value());
    EXPECT_NEAR(actual->x_m, 1.0, kTolerance);
    EXPECT_NEAR(actual->y_m, 2.0, kTolerance);
    EXPECT_NEAR(actual->z_m, 0.0, kTolerance);
    EXPECT_NEAR(actual->q.w, std::cos(kPi / 4.0), kTolerance);
    EXPECT_NEAR(actual->q.z, std::sin(kPi / 4.0), kTolerance);
}

}  // namespace
}  // namespace transforms
