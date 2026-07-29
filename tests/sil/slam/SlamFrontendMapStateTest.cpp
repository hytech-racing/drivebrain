#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "frontend/SlamFrontend.hpp"

namespace slam::frontend
{
namespace
{

SlamFrontendParams make_test_params(
    const std::size_t minimum_observations_to_confirm = 2U)
{
    SlamFrontendParams params;
    params.optimized_association_gate_m = 1.0;
    params.local_track_association_gate_m = 0.5;
    params.minimum_observations_to_confirm = minimum_observations_to_confirm;
    params.tentative_track_max_age_ns = 10'000'000'000LL;
    params.pending_track_max_age_ns = 10'000'000'000LL;
    params.minimum_detection_confidence = 0.5;
    return params;
}

MapLandmark make_landmark(const std::uint64_t landmark_id,
                          const double x_map_m = 100.0,
                          const double y_map_m = 100.0)
{
    return MapLandmark{landmark_id, transforms::Point2D{x_map_m, y_map_m}};
}

MapState make_map_state(const std::uint64_t sequence,
                        const std::int64_t timestamp_ns,
                        std::vector<MapLandmark> landmarks = {},
                        const transforms::Pose2D& pose_map_from_odom = {})
{
    MapState state;
    state.sequence = sequence;
    state.timestamp_ns = timestamp_ns;
    state.pose_map_from_odom = pose_map_from_odom;
    state.landmarks = std::move(landmarks);
    return state;
}

ConeDetection make_detection(const double x_base_m,
                             const double y_base_m = 0.0)
{
    return ConeDetection{transforms::Point2D{x_base_m, y_base_m}, 1.0};
}

ConeFrame make_frame(const std::int64_t timestamp_ns,
                     std::vector<ConeDetection> detections)
{
    ConeFrame frame;
    frame.timestamp_ns = timestamp_ns;
    frame.detections = std::move(detections);
    return frame;
}

FrontendResult promote_single_track(SlamFrontend& frontend,
                                    const std::int64_t first_timestamp_ns,
                                    const double x_base_m)
{
    const FrontendResult creation_result = frontend.process_frame(
        make_frame(first_timestamp_ns, {make_detection(x_base_m)}));
    EXPECT_TRUE(creation_result.frame_accepted);
    EXPECT_EQ(creation_result.debug.tentative_tracks_created, 1U);

    return frontend.process_frame(make_frame(
        first_timestamp_ns + 100, {make_detection(x_base_m + 0.1)}));
}

TEST(SlamFrontendMapStateTest, AcceptsFirstValidMapState)
{
    SlamFrontend frontend(make_test_params());

    const MapStateUpdateResult result =
        frontend.update_map_state(make_map_state(0U, 0));

    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(result.message, "Map state accepted");
    EXPECT_EQ(result.landmark_count, 0U);
    EXPECT_EQ(result.pending_tracks_resolved, 0U);
}

TEST(SlamFrontendMapStateTest, RejectsNonIncreasingState)
{
    SlamFrontend frontend(make_test_params());

    ASSERT_TRUE(frontend.update_map_state(make_map_state(10U, 100)).accepted);

    EXPECT_FALSE(frontend.update_map_state(make_map_state(11U, 100)).accepted);
    EXPECT_FALSE(frontend.update_map_state(make_map_state(10U, 200)).accepted);
}

TEST(SlamFrontendMapStateTest, RejectsDuplicateLandmarkIds)
{
    SlamFrontend frontend(make_test_params());

    const MapStateUpdateResult result = frontend.update_map_state(
        make_map_state(0U, 0, {make_landmark(42U), make_landmark(42U)}));

    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.message, "Map state landmark ids are not unique");
}

TEST(SlamFrontendMapStateTest, AdvancesAllocatorAboveBackendLandmarkIds)
{
    SlamFrontend frontend(make_test_params());

    ASSERT_TRUE(frontend
                    .update_map_state(make_map_state(
                        1U, 100, {make_landmark(7U), make_landmark(42U)}))
                    .accepted);

    const FrontendResult promotion = promote_single_track(frontend, 1'000, 0.0);

    ASSERT_TRUE(promotion.frame_accepted);
    ASSERT_EQ(promotion.new_landmark_ids.size(), 1U);
    EXPECT_EQ(promotion.new_landmark_ids.front(), 43U);
}

TEST(SlamFrontendMapStateTest, ResolvesAcknowledgedPendingTrack)
{
    SlamFrontend frontend(make_test_params());

    const FrontendResult promotion = promote_single_track(frontend, 100, 0.0);
    ASSERT_TRUE(promotion.frame_accepted);
    ASSERT_EQ(promotion.new_landmark_ids.size(), 1U);

    const MapStateUpdateResult result = frontend.update_map_state(
        make_map_state(1U, 300,
                       {make_landmark(promotion.new_landmark_ids.front())}));

    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(result.pending_tracks_resolved, 1U);
}

TEST(SlamFrontendMapStateTest, RejectedMapUpdateDoesNotCommitState)
{
    SlamFrontend frontend(make_test_params());

    ASSERT_TRUE(frontend.update_map_state(make_map_state(1U, 100)).accepted);

    const MapStateUpdateResult rejected = frontend.update_map_state(
        make_map_state(2U, 200, {make_landmark(7U), make_landmark(7U)}));
    ASSERT_FALSE(rejected.accepted);

    const MapStateUpdateResult corrected =
        frontend.update_map_state(make_map_state(2U, 200));
    EXPECT_TRUE(corrected.accepted);
}

TEST(SlamFrontendMapStateTest,
     RejectedAllocatorUpdateDoesNotPartiallyAdvanceAllocator)
{
    SlamFrontend frontend(make_test_params());

    ASSERT_TRUE(frontend
                    .update_map_state(make_map_state(1U, 100,
                                                     {make_landmark(42U)}))
                    .accepted);

    const MapStateUpdateResult rejected = frontend.update_map_state(
        make_map_state(2U, 200,
                       {make_landmark(500U),
                        make_landmark(
                            std::numeric_limits<std::uint64_t>::max())}));
    ASSERT_FALSE(rejected.accepted);

    const FrontendResult promotion = promote_single_track(frontend, 1'000, 0.0);
    ASSERT_TRUE(promotion.frame_accepted);
    ASSERT_EQ(promotion.new_landmark_ids.size(), 1U);
    EXPECT_EQ(promotion.new_landmark_ids.front(), 43U);
}

TEST(SlamFrontendMapStateTest, PartiallyAcknowledgesPendingTracks)
{
    SlamFrontend frontend(make_test_params());

    ASSERT_TRUE(frontend
                    .process_frame(make_frame(100, {make_detection(0.0),
                                                    make_detection(10.0)}))
                    .frame_accepted);
    const FrontendResult promotion = frontend.process_frame(
        make_frame(200, {make_detection(0.05), make_detection(10.2)}));
    ASSERT_EQ(promotion.new_landmark_ids.size(), 2U);

    // The smaller residual promotes the 0m track first, so acknowledging the
    // first ID leaves the 10m pending track active.
    const MapStateUpdateResult partial = frontend.update_map_state(
        make_map_state(1U, 300, {make_landmark(promotion.new_landmark_ids[0])}));
    ASSERT_TRUE(partial.accepted);
    EXPECT_EQ(partial.pending_tracks_resolved, 1U);

    const FrontendResult pending =
        frontend.process_frame(make_frame(400, {make_detection(10.25)}));
    ASSERT_TRUE(pending.frame_accepted);
    ASSERT_EQ(pending.landmark_observations.size(), 1U);
    EXPECT_EQ(pending.landmark_observations.front().landmark_id,
              promotion.new_landmark_ids[1]);
    EXPECT_EQ(pending.landmark_observations.front().association,
              LandmarkAssociation::PendingLandmark);
}

}  // namespace
}  // namespace slam::frontend
