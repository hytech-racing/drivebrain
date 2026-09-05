#include <gtest/gtest.h>

#include <cmath>
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
constexpr double kPi = 3.14159265358979323846264338327950288;

SlamFrontendParams make_test_params(
    const std::size_t minimum_observations_to_confirm = 3U,
    const double optimized_gate_m = 1.0, const double local_gate_m = 0.5)
{
    SlamFrontendParams params;
    params.optimized_association_gate_m = optimized_gate_m;
    params.local_track_association_gate_m = local_gate_m;
    params.minimum_observations_to_confirm = minimum_observations_to_confirm;
    params.tentative_track_max_age_ns = 10'000'000'000LL;
    params.pending_track_max_age_ns = 10'000'000'000LL;
    params.minimum_detection_confidence = 0.5;
    return params;
}

ConeDetection make_detection(const double x_base_m,
                             const double y_base_m = 0.0)
{
    return ConeDetection{transforms::Point2D{x_base_m, y_base_m}, 1.0};
}

ConeFrame make_frame(const std::int64_t timestamp_ns,
                     std::vector<ConeDetection> detections,
                     const transforms::Pose2D& pose_odom_from_base = {})
{
    ConeFrame frame;
    frame.timestamp_ns = timestamp_ns;
    frame.pose_odom_from_base = pose_odom_from_base;
    frame.detections = std::move(detections);
    return frame;
}

MapLandmark make_landmark(const std::uint64_t landmark_id,
                          const double x_map_m,
                          const double y_map_m = 0.0)
{
    return MapLandmark{landmark_id, transforms::Point2D{x_map_m, y_map_m}};
}

MapState make_map_state(const std::uint64_t sequence,
                        const std::int64_t timestamp_ns,
                        std::vector<MapLandmark> landmarks,
                        const transforms::Pose2D& pose_map_from_odom = {})
{
    MapState state;
    state.sequence = sequence;
    state.timestamp_ns = timestamp_ns;
    state.pose_map_from_odom = pose_map_from_odom;
    state.landmarks = std::move(landmarks);
    return state;
}

void expect_map_observation(const LandmarkObservation& actual,
                            const std::uint64_t expected_landmark_id,
                            const double expected_x_base_m,
                            const double expected_y_base_m,
                            const double expected_residual_m)
{
    EXPECT_EQ(actual.landmark_id, expected_landmark_id);
    EXPECT_EQ(actual.association, LandmarkAssociation::ExistingMapLandmark);
    EXPECT_NEAR(actual.measurement_base_m.x_m, expected_x_base_m, kTolerance);
    EXPECT_NEAR(actual.measurement_base_m.y_m, expected_y_base_m, kTolerance);
    EXPECT_NEAR(actual.residual_m, expected_residual_m, kTolerance);
}

TEST(SlamFrontendOptimizedAssociationTest, NoMapStateUsesTentativeTracking)
{
    SlamFrontend frontend(make_test_params());

    const FrontendResult result =
        frontend.process_frame(make_frame(100, {make_detection(5.0)}));

    ASSERT_TRUE(result.frame_accepted);
    EXPECT_TRUE(result.landmark_observations.empty());
    EXPECT_EQ(result.debug.tentative_tracks_created, 1U);
}

TEST(SlamFrontendOptimizedAssociationTest,
     NearbyMapLandmarkEmitsOriginalMeasurementAndLandmarkId)
{
    SlamFrontend frontend(make_test_params());

    ASSERT_TRUE(frontend
                    .update_map_state(make_map_state(
                        1U, 100, {make_landmark(42U, 5.0, 2.0)}))
                    .accepted);

    const FrontendResult result =
        frontend.process_frame(make_frame(200, {make_detection(5.2, 1.9)}));

    ASSERT_TRUE(result.frame_accepted);
    ASSERT_EQ(result.landmark_observations.size(), 1U);
    expect_map_observation(result.landmark_observations.front(), 42U, 5.2, 1.9,
                           std::hypot(0.2, -0.1));
    EXPECT_TRUE(result.new_landmark_ids.empty());
}

TEST(SlamFrontendOptimizedAssociationTest,
     AppliesMapFromOdomAndOdomFromBaseTransforms)
{
    SlamFrontend frontend(make_test_params());

    const transforms::Pose2D pose_map_from_odom{10.0, 5.0, kPi / 2.0};
    ASSERT_TRUE(frontend
                    .update_map_state(make_map_state(
                        1U, 100, {make_landmark(77U, 11.0, 10.0)},
                        pose_map_from_odom))
                    .accepted);

    const transforms::Pose2D pose_odom_from_base{2.0, 0.0, 0.0};
    const FrontendResult result = frontend.process_frame(
        make_frame(200, {make_detection(3.2, -1.0)}, pose_odom_from_base));

    ASSERT_TRUE(result.frame_accepted);
    ASSERT_EQ(result.landmark_observations.size(), 1U);
    expect_map_observation(result.landmark_observations.front(), 77U, 3.2,
                           -1.0, 0.2);
}

TEST(SlamFrontendOptimizedAssociationTest,
     OutOfGateMapDetectionFallsThroughToTentativeTracking)
{
    SlamFrontend frontend(make_test_params(3U, 1.0));

    ASSERT_TRUE(frontend
                    .update_map_state(make_map_state(
                        1U, 100, {make_landmark(7U, 5.0)}))
                    .accepted);

    const FrontendResult result =
        frontend.process_frame(make_frame(200, {make_detection(6.01)}));

    ASSERT_TRUE(result.frame_accepted);
    EXPECT_TRUE(result.landmark_observations.empty());
    EXPECT_EQ(result.debug.tentative_tracks_created, 1U);
    EXPECT_EQ(result.debug.unmatched_detection_count, 1U);
}

TEST(SlamFrontendOptimizedAssociationTest,
     ClosestDetectionWinsWhenTwoDetectionsCompeteForOneLandmark)
{
    SlamFrontend frontend(make_test_params());

    ASSERT_TRUE(frontend
                    .update_map_state(make_map_state(
                        1U, 100, {make_landmark(50U, 5.0)}))
                    .accepted);

    const FrontendResult result = frontend.process_frame(
        make_frame(200, {make_detection(5.1), make_detection(5.4)}));

    ASSERT_TRUE(result.frame_accepted);
    ASSERT_EQ(result.landmark_observations.size(), 1U);
    expect_map_observation(result.landmark_observations.front(), 50U, 5.1, 0.0,
                           0.1);
    EXPECT_EQ(result.debug.tentative_tracks_created, 1U);
    EXPECT_EQ(result.debug.unmatched_detection_count, 1U);
}

TEST(SlamFrontendOptimizedAssociationTest,
     ClosestLandmarkWinsWhenOneDetectionCompetesForTwoLandmarks)
{
    SlamFrontend frontend(make_test_params());

    ASSERT_TRUE(frontend
                    .update_map_state(make_map_state(
                        1U, 100,
                        {make_landmark(10U, 5.4), make_landmark(20U, 5.1)}))
                    .accepted);

    const FrontendResult result =
        frontend.process_frame(make_frame(200, {make_detection(5.0)}));

    ASSERT_TRUE(result.frame_accepted);
    ASSERT_EQ(result.landmark_observations.size(), 1U);
    expect_map_observation(result.landmark_observations.front(), 20U, 5.0, 0.0,
                           0.1);
    EXPECT_EQ(result.debug.tentative_tracks_created, 0U);
}

TEST(SlamFrontendOptimizedAssociationTest,
     OptimizedAssociationConsumesDetectionBeforePendingTrack)
{
    SlamFrontend frontend(make_test_params(2U, 1.0, 0.5));

    ASSERT_TRUE(frontend.process_frame(make_frame(100, {make_detection(5.0)}))
                    .frame_accepted);
    const FrontendResult promotion =
        frontend.process_frame(make_frame(200, {make_detection(5.1)}));
    ASSERT_EQ(promotion.new_landmark_ids.size(), 1U);

    ASSERT_TRUE(frontend
                    .update_map_state(make_map_state(
                        1U, 250, {make_landmark(10U, 5.2)}))
                    .accepted);

    const FrontendResult result =
        frontend.process_frame(make_frame(300, {make_detection(5.2)}));

    ASSERT_TRUE(result.frame_accepted);
    ASSERT_EQ(result.landmark_observations.size(), 1U);
    expect_map_observation(result.landmark_observations.front(), 10U, 5.2, 0.0,
                           0.0);
    EXPECT_EQ(result.debug.pending_associations, 0U);
    EXPECT_EQ(result.debug.tentative_tracks_created, 0U);
}

TEST(SlamFrontendOptimizedAssociationTest,
     FilteredDetectionBeforeOptimizedMatchKeepsIndicesStraight)
{
    SlamFrontend frontend(make_test_params());

    ASSERT_TRUE(frontend
                    .update_map_state(make_map_state(
                        1U, 100, {make_landmark(42U, 8.0, 1.0)}))
                    .accepted);

    const ConeDetection invalid_detection{
        transforms::Point2D{std::numeric_limits<double>::quiet_NaN(), 0.0},
        1.0};

    const FrontendResult result = frontend.process_frame(
        make_frame(200, {invalid_detection, make_detection(8.2, 1.0)}));

    ASSERT_TRUE(result.frame_accepted);
    EXPECT_EQ(result.debug.detections_received, 2U);
    EXPECT_EQ(result.debug.detections_rejected_invalid, 1U);
    ASSERT_EQ(result.landmark_observations.size(), 1U);
    expect_map_observation(result.landmark_observations.front(), 42U, 8.2, 1.0,
                           0.2);
}

}  // namespace
}  // namespace slam::frontend
