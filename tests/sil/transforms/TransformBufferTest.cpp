#include <gtest/gtest.h>

#include <FrameId.hpp>
#include <RigidTransform2D.hpp>
#include <TransformBuffer.hpp>
#include <cmath>
#include <limits>
#include <optional>

namespace transforms
{
namespace
{

constexpr double kTolerance = 1e-9;
constexpr double kPi = 3.14159265358979323846;

void expect_transform_near(const RigidTransform2D& actual,
                           const RigidTransform2D& expected)
{
    EXPECT_NEAR(actual.x_m, expected.x_m, kTolerance);
    EXPECT_NEAR(actual.y_m, expected.y_m, kTolerance);
    EXPECT_NEAR(actual.yaw_rad, expected.yaw_rad, kTolerance);
}

TEST(TransformBufferTest, ExactTimestampLookup)
{
    TransformBuffer buffer(1000);
    const RigidTransform2D expected{1.0, 2.0, 0.3};

    EXPECT_TRUE(buffer.insert_T_odom_base(100, expected));
    EXPECT_TRUE(buffer.insert_T_odom_base(200, {3.0, 4.0, 0.6}));

    const std::optional<RigidTransform2D> actual =
        buffer.lookup(FrameId::Odom, FrameId::Baselink, 100);

    ASSERT_TRUE(actual.has_value());
    expect_transform_near(*actual, expected);
}

TEST(TransformBufferTest, MidpointTranslationInterpolation)
{
    TransformBuffer buffer(1000);

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {0.0, 2.0, 0.0}));
    EXPECT_TRUE(buffer.insert_T_odom_base(200, {10.0, 6.0, 0.0}));

    const std::optional<RigidTransform2D> actual =
        buffer.lookup(FrameId::Odom, FrameId::Baselink, 150);

    ASSERT_TRUE(actual.has_value());
    expect_transform_near(*actual, {5.0, 4.0, 0.0});
}

TEST(TransformBufferTest, YawInterpolationAcrossPiBoundary)
{
    TransformBuffer buffer(1000);

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {0.0, 0.0, kPi - 0.1}));
    EXPECT_TRUE(buffer.insert_T_odom_base(200, {0.0, 0.0, -kPi + 0.1}));

    const std::optional<RigidTransform2D> actual =
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

TEST(TransformBufferTest, HistoryPruningBySensorTimestamp)
{
    TransformBuffer buffer(100);

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {1.0, 0.0, 0.0}));
    EXPECT_TRUE(buffer.insert_T_odom_base(150, {2.0, 0.0, 0.0}));
    EXPECT_TRUE(buffer.insert_T_odom_base(225, {3.0, 0.0, 0.0}));

    EXPECT_EQ(buffer.lookup(FrameId::Odom, FrameId::Baselink, 100),
              std::nullopt);

    const std::optional<RigidTransform2D> retained =
        buffer.lookup(FrameId::Odom, FrameId::Baselink, 150);
    ASSERT_TRUE(retained.has_value());
    expect_transform_near(*retained, {2.0, 0.0, 0.0});
}

TEST(TransformBufferTest, OutOfOrderInsertionRejected)
{
    TransformBuffer buffer(1000);
    const RigidTransform2D original_latest{2.0, 0.0, 0.0};

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {1.0, 0.0, 0.0}));
    EXPECT_TRUE(buffer.insert_T_odom_base(200, original_latest));
    EXPECT_FALSE(buffer.insert_T_odom_base(150, {9.0, 0.0, 0.0}));

    const std::optional<RigidTransform2D> latest =
        buffer.lookup(FrameId::Odom, FrameId::Baselink, 200);
    ASSERT_TRUE(latest.has_value());
    expect_transform_near(*latest, original_latest);
}

TEST(TransformBufferTest, SameTimestampInsertionReplacesExistingSample)
{
    TransformBuffer buffer(1000);
    const RigidTransform2D replacement{2.0, 3.0, 0.4};

    EXPECT_TRUE(buffer.insert_T_odom_base(100, {1.0, 0.0, 0.0}));
    EXPECT_TRUE(buffer.insert_T_odom_base(100, replacement));

    const std::optional<RigidTransform2D> actual =
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

    EXPECT_EQ(buffer.lookup(FrameId::Odom, FrameId::Baselink, 100),
              std::nullopt);
    EXPECT_EQ(buffer.lookup(FrameId::Map, FrameId::Odom, 100), std::nullopt);
}

TEST(TransformBufferTest, MapOdomLookupHoldsLatestSampleAtOrBeforeQuery)
{
    TransformBuffer buffer(1000);
    const RigidTransform2D before{1.0, 2.0, 0.3};
    const RigidTransform2D after{10.0, 20.0, 1.3};

    EXPECT_TRUE(buffer.insert_T_map_odom(100, before));
    EXPECT_TRUE(buffer.insert_T_map_odom(200, after));

    const std::optional<RigidTransform2D> actual =
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

}  // namespace
}  // namespace transforms
