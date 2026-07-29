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

constexpr double kTolerance = 1e-9;

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

ConeDetection make_detection(const double x_base_m,
                             const double y_base_m = 0.0,
                             const double confidence = 1.0)
{
    return ConeDetection{transforms::Point2D{x_base_m, y_base_m}, confidence};
}

ConeFrame make_frame(const std::int64_t timestamp_ns,
                     std::vector<ConeDetection> detections)
{
    ConeFrame frame;
    frame.timestamp_ns = timestamp_ns;
    frame.detections = std::move(detections);
    return frame;
}

void expect_observation(const LandmarkObservation& actual,
                        const std::uint64_t expected_landmark_id,
                        const double expected_x_base_m,
                        const LandmarkAssociation expected_association)
{
    EXPECT_EQ(actual.landmark_id, expected_landmark_id);
    EXPECT_EQ(actual.association, expected_association);
    EXPECT_NEAR(actual.measurement_base_m.x_m, expected_x_base_m, kTolerance);
    EXPECT_NEAR(actual.measurement_base_m.y_m, 0.0, kTolerance);
}

TEST(SlamFrontendTrackingTest, FilteredDetectionKeepsSourceIndexStable)
{
    SlamFrontend frontend(make_test_params(3U));

    const ConeDetection invalid_detection{
        transforms::Point2D{std::numeric_limits<double>::quiet_NaN(), 0.0},
        1.0};

    ASSERT_TRUE(frontend.process_frame(make_frame(
                            100, {invalid_detection, make_detection(10.0)}))
                    .frame_accepted);

    const FrontendResult second_result = frontend.process_frame(
        make_frame(200, {invalid_detection, make_detection(10.1)}));

    ASSERT_TRUE(second_result.frame_accepted);
    EXPECT_EQ(second_result.debug.detections_received, 2U);
    EXPECT_EQ(second_result.debug.detections_rejected_invalid, 1U);
    EXPECT_EQ(second_result.debug.tentative_associations, 1U);
    EXPECT_TRUE(second_result.new_landmark_ids.empty());

    const FrontendResult third_result = frontend.process_frame(
        make_frame(300, {invalid_detection, make_detection(10.2)}));

    ASSERT_TRUE(third_result.frame_accepted);
    ASSERT_EQ(third_result.new_landmark_ids.size(), 1U);
    ASSERT_EQ(third_result.landmark_observations.size(), 1U);
    expect_observation(third_result.landmark_observations.front(), 0U, 10.2,
                       LandmarkAssociation::NewLandmark);
}

TEST(SlamFrontendTrackingTest, PendingAssociationsConsumeDetectionsFirst)
{
    SlamFrontend frontend(make_test_params(2U));

    ASSERT_TRUE(frontend.process_frame(make_frame(100, {make_detection(5.0)}))
                    .frame_accepted);

    const FrontendResult promotion =
        frontend.process_frame(make_frame(200, {make_detection(5.1)}));
    ASSERT_EQ(promotion.new_landmark_ids.size(), 1U);

    const FrontendResult pending =
        frontend.process_frame(make_frame(300, {make_detection(5.2)}));

    ASSERT_TRUE(pending.frame_accepted);
    EXPECT_EQ(pending.debug.pending_associations, 1U);
    EXPECT_EQ(pending.debug.tentative_associations, 0U);
    EXPECT_EQ(pending.debug.tentative_tracks_created, 0U);
    ASSERT_EQ(pending.landmark_observations.size(), 1U);
    EXPECT_EQ(pending.landmark_observations.front().association,
              LandmarkAssociation::PendingLandmark);
}

TEST(SlamFrontendTrackingTest,
     PendingTargetViewIndexMapsBackToInterleavedLocalTrack)
{
    SlamFrontend frontend(make_test_params(2U));

    ASSERT_TRUE(frontend
                    .process_frame(make_frame(100,
                                              {make_detection(0.0),
                                               make_detection(10.0),
                                               make_detection(20.0),
                                               make_detection(30.0)}))
                    .frame_accepted);

    const FrontendResult promotion = frontend.process_frame(
        make_frame(200, {make_detection(0.1), make_detection(20.1)}));
    ASSERT_TRUE(promotion.frame_accepted);
    ASSERT_EQ(promotion.new_landmark_ids.size(), 2U);
    EXPECT_EQ(promotion.new_landmark_ids.at(0), 0U);
    EXPECT_EQ(promotion.new_landmark_ids.at(1), 1U);

    const FrontendResult pending =
        frontend.process_frame(make_frame(300, {make_detection(20.2)}));

    ASSERT_TRUE(pending.frame_accepted);
    EXPECT_EQ(pending.debug.pending_associations, 1U);
    EXPECT_EQ(pending.debug.tentative_tracks_created, 0U);
    ASSERT_EQ(pending.landmark_observations.size(), 1U);
    expect_observation(pending.landmark_observations.front(), 1U, 20.2,
                       LandmarkAssociation::PendingLandmark);
}

TEST(SlamFrontendTrackingTest,
     TentativeTargetViewIndexMapsBackToInterleavedLocalTrackDuringPromotion)
{
    SlamFrontend frontend(make_test_params(2U));

    ASSERT_TRUE(frontend
                    .process_frame(make_frame(100,
                                              {make_detection(0.0),
                                               make_detection(10.0),
                                               make_detection(20.0),
                                               make_detection(30.0)}))
                    .frame_accepted);

    const FrontendResult first_promotion = frontend.process_frame(
        make_frame(200, {make_detection(0.1), make_detection(20.1)}));
    ASSERT_TRUE(first_promotion.frame_accepted);
    ASSERT_EQ(first_promotion.new_landmark_ids.size(), 2U);

    const FrontendResult second_promotion = frontend.process_frame(
        make_frame(300, {make_detection(10.1), make_detection(30.1)}));

    ASSERT_TRUE(second_promotion.frame_accepted);
    EXPECT_EQ(second_promotion.debug.tentative_associations, 2U);
    EXPECT_EQ(second_promotion.debug.tracks_promoted, 2U);
    ASSERT_EQ(second_promotion.new_landmark_ids.size(), 2U);
    EXPECT_EQ(second_promotion.new_landmark_ids.at(0), 2U);
    EXPECT_EQ(second_promotion.new_landmark_ids.at(1), 3U);
    ASSERT_EQ(second_promotion.landmark_observations.size(), 2U);
    expect_observation(second_promotion.landmark_observations.at(0), 2U, 10.1,
                       LandmarkAssociation::NewLandmark);
    expect_observation(second_promotion.landmark_observations.at(1), 3U, 30.1,
                       LandmarkAssociation::NewLandmark);
}

TEST(SlamFrontendTrackingTest, StaleTentativeTrackIsRemoved)
{
    SlamFrontendParams params = make_test_params(3U);
    params.tentative_track_max_age_ns = 50;
    SlamFrontend frontend(params);

    ASSERT_TRUE(frontend.process_frame(make_frame(100, {make_detection(5.0)}))
                    .frame_accepted);

    const FrontendResult result =
        frontend.process_frame(make_frame(200, {make_detection(50.0)}));

    ASSERT_TRUE(result.frame_accepted);
    EXPECT_EQ(result.debug.tracks_removed_stale, 1U);
    EXPECT_EQ(result.debug.tentative_tracks_created, 1U);
}

TEST(SlamFrontendTrackingTest, StalePendingTrackIsRemoved)
{
    SlamFrontendParams params = make_test_params(2U);
    params.pending_track_max_age_ns = 50;
    SlamFrontend frontend(params);

    ASSERT_TRUE(frontend.process_frame(make_frame(100, {make_detection(5.0)}))
                    .frame_accepted);
    const FrontendResult promotion =
        frontend.process_frame(make_frame(200, {make_detection(5.1)}));
    ASSERT_EQ(promotion.new_landmark_ids.size(), 1U);

    const FrontendResult result =
        frontend.process_frame(make_frame(300, {make_detection(50.0)}));

    ASSERT_TRUE(result.frame_accepted);
    EXPECT_EQ(result.debug.tracks_removed_stale, 1U);
    EXPECT_EQ(result.debug.tentative_tracks_created, 1U);
}

TEST(SlamFrontendTrackingTest, RejectedFrameDoesNotConsumeTimestamp)
{
    SlamFrontend frontend(make_test_params());

    const ConeDetection invalid_detection{
        transforms::Point2D{std::numeric_limits<double>::quiet_NaN(), 0.0},
        1.0};

    ConeFrame rejected_frame = make_frame(100, {invalid_detection});
    rejected_frame.pose_odom_from_base.x_m =
        std::numeric_limits<double>::quiet_NaN();

    const FrontendResult invalid = frontend.process_frame(rejected_frame);
    EXPECT_FALSE(invalid.frame_accepted);

    const FrontendResult accepted =
        frontend.process_frame(make_frame(100, {make_detection(5.0)}));
    EXPECT_TRUE(accepted.frame_accepted);
    EXPECT_EQ(accepted.debug.tentative_tracks_created, 1U);
}

}  // namespace
}  // namespace slam::frontend
