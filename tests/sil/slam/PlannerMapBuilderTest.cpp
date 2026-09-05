#include <gtest/gtest.h>

#include <unordered_map>

#include "frontend/PlannerMapBuilder.hpp"

namespace slam::frontend
{
namespace
{

MapState make_map_state()
{
    MapState state;
    state.sequence = 10U;
    state.timestamp_ns = 1000;
    state.pose_map_from_odom = transforms::Pose2D{10.0, 20.0, 0.5};
    return state;
}

TEST(PlannerMapBuilderTest, BuildsOptimizedLandmarksWithColor)
{
    MapState state = make_map_state();
    state.landmarks.push_back(MapLandmark{2U, transforms::Point2D{1.0, 2.0}});

    std::unordered_map<std::uint64_t, ColorEvidence> color_by_id;
    color_by_id[2U].yellow = 0.75;

    const PlannerMap map = PlannerMapBuilder{}.build(1U, 2000, state, {},
                                                      color_by_id);

    ASSERT_EQ(map.landmarks.size(), 1U);
    EXPECT_EQ(map.landmarks.front().landmark_id, 2U);
    EXPECT_EQ(map.landmarks.front().state, PlannerLandmarkState::Optimized);
    EXPECT_EQ(map.landmarks.front().color, ConeColor::Yellow);
    EXPECT_DOUBLE_EQ(map.landmarks.front().color_confidence, 1.0);
}

TEST(PlannerMapBuilderTest, TransformsPendingLandmarksIntoMap)
{
    MapState state = make_map_state();
    state.pose_map_from_odom = transforms::Pose2D{10.0, 20.0, 0.0};

    const PendingPlannerLandmark pending{
        3U,
        transforms::Point2D{1.0, 2.0},
        LandmarkColorEstimate{ConeColor::Blue, 0.9},
    };

    const PlannerMap map = PlannerMapBuilder{}.build(1U, 2000, state,
                                                      {pending}, {});

    ASSERT_EQ(map.landmarks.size(), 1U);
    EXPECT_EQ(map.landmarks.front().landmark_id, 3U);
    EXPECT_EQ(map.landmarks.front().state, PlannerLandmarkState::Pending);
    EXPECT_NEAR(map.landmarks.front().position_map_m.x_m, 11.0, 1e-9);
    EXPECT_NEAR(map.landmarks.front().position_map_m.y_m, 22.0, 1e-9);
    EXPECT_EQ(map.landmarks.front().color, ConeColor::Blue);
}

TEST(PlannerMapBuilderTest, OptimizedLandmarkWinsOverPendingSameIdAndSorts)
{
    MapState state = make_map_state();
    state.landmarks.push_back(MapLandmark{5U, transforms::Point2D{5.0, 0.0}});
    state.landmarks.push_back(MapLandmark{1U, transforms::Point2D{1.0, 0.0}});

    const PendingPlannerLandmark duplicate_pending{
        5U,
        transforms::Point2D{50.0, 0.0},
        LandmarkColorEstimate{ConeColor::Blue, 1.0},
    };

    const PlannerMap map = PlannerMapBuilder{}.build(
        1U, 2000, state, {duplicate_pending}, {});

    ASSERT_EQ(map.landmarks.size(), 2U);
    EXPECT_EQ(map.landmarks.at(0).landmark_id, 1U);
    EXPECT_EQ(map.landmarks.at(1).landmark_id, 5U);
    EXPECT_EQ(map.landmarks.at(1).state, PlannerLandmarkState::Optimized);
    EXPECT_NEAR(map.landmarks.at(1).position_map_m.x_m, 5.0, 1e-9);
}

}  // namespace
}  // namespace slam::frontend
