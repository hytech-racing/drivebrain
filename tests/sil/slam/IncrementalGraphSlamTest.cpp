#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include "backend/IncrementalGraphSlam.hpp"

namespace slam::backend
{
namespace
{

constexpr double kPoseTolerance = 1e-5;
constexpr double kPointTolerance = 1e-5;

IncrementalGraphSlamParams make_test_params()
{
    IncrementalGraphSlamParams params;
    params.prior_pose_noise.x_std_m = 0.01;
    params.prior_pose_noise.y_std_m = 0.01;
    params.prior_pose_noise.yaw_std_rad = 0.01;
    params.odom_pose_noise.x_std_m = 0.10;
    params.odom_pose_noise.y_std_m = 0.10;
    params.odom_pose_noise.yaw_std_rad = 0.02;
    params.landmark_measurement_noise.bearing_std_rad = 0.05;
    params.landmark_measurement_noise.range_std_m = 0.30;
    params.measurement_range.min_m = 0.10;
    params.measurement_range.max_m = 50.0;
    params.isam2.relinearization_threshold = 0.01;
    params.isam2.relinearization_skip = 1U;
    params.isam2.evaluate_nonlinear_error = true;
    return params;
}

LandmarkObservation make_observation(const std::uint64_t landmark_id,
                                     const double x_base_m,
                                     const double y_base_m,
                                     const LandmarkAssociation association =
                                         LandmarkAssociation::NewLandmark)
{
    LandmarkObservation observation;
    observation.landmark_id = landmark_id;
    observation.measurement_base_m = transforms::Point2D{x_base_m, y_base_m};
    observation.association = association;
    return observation;
}

LandmarkFrame make_frame(const std::uint64_t frame_index,
                         const std::int64_t timestamp_ns,
                         const transforms::Pose2D& recorded_pose,
                         std::vector<LandmarkObservation> observations = {})
{
    LandmarkFrame frame;
    frame.frame_index = frame_index;
    frame.timestamp_ns = timestamp_ns;
    frame.recorded_pose_odom_from_base = recorded_pose;
    frame.observations = std::move(observations);
    return frame;
}

void expect_pose_near(const transforms::Pose2D& actual,
                      const transforms::Pose2D& expected,
                      const double tolerance = kPoseTolerance)
{
    EXPECT_NEAR(actual.x_m, expected.x_m, tolerance);
    EXPECT_NEAR(actual.y_m, expected.y_m, tolerance);
    EXPECT_NEAR(std::atan2(std::sin(actual.yaw_rad - expected.yaw_rad),
                           std::cos(actual.yaw_rad - expected.yaw_rad)),
                0.0, tolerance);
}

void expect_point_near(const transforms::Point2D& actual,
                       const transforms::Point2D& expected,
                       const double tolerance = kPointTolerance)
{
    EXPECT_NEAR(actual.x_m, expected.x_m, tolerance);
    EXPECT_NEAR(actual.y_m, expected.y_m, tolerance);
}

void expect_reconstructed_map_pose(const IncrementalPoseResult& pose)
{
    const transforms::Pose2D reconstructed =
        pose.pose_map_from_odom.compose(pose.recorded_pose_odom_from_base);
    expect_pose_near(reconstructed, pose.optimized_pose_map_from_base);
}

const LandmarkEstimate* find_landmark(
    const IncrementalGraphSlamSnapshot& snapshot,
    const std::uint64_t landmark_id)
{
    const auto iterator = std::find_if(
        snapshot.landmarks.begin(), snapshot.landmarks.end(),
        [landmark_id](const LandmarkEstimate& estimate)
        { return estimate.landmark_id == landmark_id; });

    return iterator == snapshot.landmarks.end() ? nullptr : &(*iterator);
}

TEST(IncrementalGraphSlamTest, InvalidParametersThrowAtConstruction)
{
    IncrementalGraphSlamParams params = make_test_params();
    params.isam2.relinearization_skip = 0U;

    EXPECT_THROW(IncrementalGraphSlam slam(params), std::invalid_argument);
}

TEST(IncrementalGraphSlamTest, RejectsInvalidNoiseAndRangeParameters)
{
    {
        IncrementalGraphSlamParams params = make_test_params();
        params.prior_pose_noise.x_std_m = 0.0;
        EXPECT_THROW(IncrementalGraphSlam slam(params), std::invalid_argument);
    }
    {
        IncrementalGraphSlamParams params = make_test_params();
        params.odom_pose_noise.y_std_m = -1.0;
        EXPECT_THROW(IncrementalGraphSlam slam(params), std::invalid_argument);
    }
    {
        IncrementalGraphSlamParams params = make_test_params();
        params.landmark_measurement_noise.range_std_m =
            std::numeric_limits<double>::quiet_NaN();
        EXPECT_THROW(IncrementalGraphSlam slam(params), std::invalid_argument);
    }
    {
        IncrementalGraphSlamParams params = make_test_params();
        params.measurement_range.min_m = params.measurement_range.max_m;
        EXPECT_THROW(IncrementalGraphSlam slam(params), std::invalid_argument);
    }
    {
        IncrementalGraphSlamParams params = make_test_params();
        params.isam2.relinearization_threshold = 0.0;
        EXPECT_THROW(IncrementalGraphSlam slam(params), std::invalid_argument);
    }
}

TEST(IncrementalGraphSlamTest, EmptySnapshotIsSuccessful)
{
    const IncrementalGraphSlam slam(make_test_params());

    const IncrementalGraphSlamSnapshot snapshot = slam.snapshot();

    EXPECT_TRUE(snapshot.success);
    EXPECT_TRUE(snapshot.poses.empty());
    EXPECT_TRUE(snapshot.landmarks.empty());
    EXPECT_FALSE(snapshot.latest_pose_map_from_odom.has_value());
}

TEST(IncrementalGraphSlamTest, FirstFrameCreatesPoseZeroPriorAndMapReference)
{
    IncrementalGraphSlam slam(make_test_params());

    const LandmarkFrame frame =
        make_frame(17U, 1'000, transforms::Pose2D{12.0, -3.0, 0.8});

    const IncrementalGraphSlamResult result = slam.process_frame(frame);

    ASSERT_TRUE(result.debug.frame_accepted);
    ASSERT_TRUE(result.debug.update_success) << result.debug.message;
    ASSERT_TRUE(result.current_pose.has_value());

    const IncrementalPoseResult& pose = result.current_pose.value();
    EXPECT_EQ(pose.pose_index, 0U);
    EXPECT_EQ(pose.frame_index, 17U);
    EXPECT_EQ(pose.timestamp_ns, 1'000);
    expect_pose_near(pose.recorded_pose_odom_from_base,
                     transforms::Pose2D{12.0, -3.0, 0.8});
    expect_pose_near(pose.initial_pose_map_from_base, transforms::Pose2D{});
    expect_pose_near(pose.optimized_pose_map_from_base, transforms::Pose2D{});
    expect_reconstructed_map_pose(pose);
}

TEST(IncrementalGraphSlamTest, SecondFrameUsesRelativeOdometryWithRotation)
{
    IncrementalGraphSlam slam(make_test_params());

    const IncrementalGraphSlamResult first = slam.process_frame(make_frame(
        10U, 100, transforms::Pose2D{10.0, 5.0, 1.5707963267948966}));
    const IncrementalGraphSlamResult second = slam.process_frame(make_frame(
        50U, 200, transforms::Pose2D{10.0, 7.0, 1.6707963267948966}));

    ASSERT_TRUE(first.debug.update_success) << first.debug.message;
    ASSERT_TRUE(second.debug.update_success) << second.debug.message;
    ASSERT_TRUE(second.current_pose.has_value());

    const IncrementalPoseResult& pose = second.current_pose.value();
    EXPECT_EQ(pose.pose_index, 1U);
    EXPECT_EQ(pose.frame_index, 50U);
    expect_pose_near(pose.initial_pose_map_from_base,
                     transforms::Pose2D{2.0, 0.0, 0.1});
    expect_reconstructed_map_pose(pose);
}

TEST(IncrementalGraphSlamTest, RepeatedLandmarkAddsFactorsWithoutReinit)
{
    IncrementalGraphSlam slam(make_test_params());

    const IncrementalGraphSlamResult first = slam.process_frame(make_frame(
        0U, 100, transforms::Pose2D{0.0, 0.0, 0.0},
        {make_observation(42U, 4.0, 2.0)}));
    const IncrementalGraphSlamResult second = slam.process_frame(make_frame(
        1U, 200, transforms::Pose2D{1.0, 0.0, 0.0},
        {make_observation(42U, 3.0, 2.0,
                          LandmarkAssociation::ExistingMapLandmark)}));

    ASSERT_TRUE(first.debug.update_success) << first.debug.message;
    ASSERT_TRUE(second.debug.update_success) << second.debug.message;
    EXPECT_EQ(first.debug.update.new_landmarks_added, 1U);
    EXPECT_EQ(second.debug.update.new_landmarks_added, 0U);
    EXPECT_TRUE(second.new_landmarks.empty());

    const IncrementalGraphSlamSnapshot snapshot = slam.snapshot();
    ASSERT_TRUE(snapshot.success) << snapshot.message;
    const LandmarkEstimate* landmark = find_landmark(snapshot, 42U);
    ASSERT_NE(landmark, nullptr);
    expect_point_near(landmark->initial_position_map,
                      transforms::Point2D{4.0, 2.0});
}

TEST(IncrementalGraphSlamTest, RotatedInitialPoseInitializesLandmarkInMapFrame)
{
    IncrementalGraphSlam slam(make_test_params());

    const IncrementalGraphSlamResult result = slam.process_frame(make_frame(
        0U, 100, transforms::Pose2D{10.0, 0.0, 1.5707963267948966},
        {make_observation(42U, 2.0, 0.0)}));

    ASSERT_TRUE(result.debug.update_success) << result.debug.message;

    const IncrementalGraphSlamSnapshot snapshot = slam.snapshot();
    ASSERT_TRUE(snapshot.success) << snapshot.message;
    const LandmarkEstimate* landmark = find_landmark(snapshot, 42U);
    ASSERT_NE(landmark, nullptr);

    // First recorded pose defines map identity, so base-frame measurement is
    // inserted directly in map frame after rebasing.
    expect_point_near(landmark->initial_position_map,
                      transforms::Point2D{2.0, 0.0});
}

TEST(IncrementalGraphSlamTest, DifferentIdsAtSameMeasurementStayDistinct)
{
    IncrementalGraphSlam slam(make_test_params());

    const IncrementalGraphSlamResult result = slam.process_frame(make_frame(
        0U, 100, transforms::Pose2D{},
        {make_observation(10U, 4.0, 1.0), make_observation(900U, 4.0, 1.0)}));

    ASSERT_TRUE(result.debug.update_success) << result.debug.message;
    EXPECT_EQ(result.debug.update.observations_admitted, 2U);
    EXPECT_EQ(result.debug.update.new_landmarks_added, 2U);

    const IncrementalGraphSlamSnapshot snapshot = slam.snapshot();
    ASSERT_TRUE(snapshot.success) << snapshot.message;
    EXPECT_NE(find_landmark(snapshot, 10U), nullptr);
    EXPECT_NE(find_landmark(snapshot, 900U), nullptr);
}

TEST(IncrementalGraphSlamTest, UnseenExistingLandmarkIsRejected)
{
    IncrementalGraphSlam slam(make_test_params());

    const IncrementalGraphSlamResult result = slam.process_frame(make_frame(
        0U, 100, transforms::Pose2D{},
        {make_observation(42U, 4.0, 2.0,
                          LandmarkAssociation::ExistingMapLandmark)}));

    ASSERT_TRUE(result.debug.update_success) << result.debug.message;
    EXPECT_EQ(result.debug.update.observations_admitted, 0U);
    EXPECT_EQ(result.debug.update.new_landmarks_added, 0U);
    EXPECT_EQ(result.debug.update.observations_rejected.unconfirmed_landmark_id,
              1U);
    EXPECT_TRUE(result.new_landmarks.empty());

    const IncrementalGraphSlamSnapshot snapshot = slam.snapshot();
    ASSERT_TRUE(snapshot.success) << snapshot.message;
    EXPECT_EQ(find_landmark(snapshot, 42U), nullptr);
}

TEST(IncrementalGraphSlamTest, UnseenPendingLandmarkIsRejected)
{
    IncrementalGraphSlam slam(make_test_params());

    const IncrementalGraphSlamResult result = slam.process_frame(make_frame(
        0U, 100, transforms::Pose2D{},
        {make_observation(42U, 4.0, 2.0,
                          LandmarkAssociation::PendingLandmark)}));

    ASSERT_TRUE(result.debug.update_success) << result.debug.message;
    EXPECT_EQ(result.debug.update.observations_admitted, 0U);
    EXPECT_EQ(result.debug.update.new_landmarks_added, 0U);
    EXPECT_EQ(result.debug.update.observations_rejected.unconfirmed_landmark_id,
              1U);
}

TEST(IncrementalGraphSlamTest, RepeatedNewLandmarkAfterInitializationIsRejected)
{
    IncrementalGraphSlam slam(make_test_params());

    const IncrementalGraphSlamResult first = slam.process_frame(make_frame(
        0U, 100, transforms::Pose2D{}, {make_observation(42U, 4.0, 2.0)}));
    const IncrementalGraphSlamResult second = slam.process_frame(make_frame(
        1U, 200, transforms::Pose2D{1.0, 0.0, 0.0},
        {make_observation(42U, 3.0, 2.0)}));

    ASSERT_TRUE(first.debug.update_success) << first.debug.message;
    ASSERT_TRUE(second.debug.update_success) << second.debug.message;
    EXPECT_EQ(second.debug.update.observations_admitted, 0U);
    EXPECT_EQ(second.debug.update.new_landmarks_added, 0U);
    EXPECT_EQ(second.debug.update.observations_rejected.duplicate_landmark_id,
              1U);
}

TEST(IncrementalGraphSlamTest, RejectsDuplicateAndInvalidObservations)
{
    IncrementalGraphSlamParams params = make_test_params();
    params.measurement_range.min_m = 0.5;
    params.measurement_range.max_m = 5.0;
    IncrementalGraphSlam slam(params);

    const double infinity = std::numeric_limits<double>::infinity();
    const IncrementalGraphSlamResult result = slam.process_frame(make_frame(
        0U, 100, transforms::Pose2D{},
        {make_observation(10U, 3.0, 4.0), make_observation(10U, 4.0, 0.0),
         make_observation(20U, infinity, 0.0),
         make_observation(30U, 6.0, 0.0)}));

    ASSERT_TRUE(result.debug.update_success) << result.debug.message;
    EXPECT_EQ(result.debug.update.observations_admitted, 1U);
    EXPECT_EQ(result.debug.update.observations_rejected.duplicate_landmark_id,
              1U);
    EXPECT_EQ(result.debug.update.observations_rejected.nonfinite_measurement,
              1U);
    EXPECT_EQ(
        result.debug.update.observations_rejected.outside_measurement_range,
        1U);
}

TEST(IncrementalGraphSlamTest, NonIncreasingTimestampDoesNotCommitState)
{
    IncrementalGraphSlam slam(make_test_params());

    const IncrementalGraphSlamResult first =
        slam.process_frame(make_frame(5U, 100, transforms::Pose2D{}));
    ASSERT_TRUE(first.debug.update_success) << first.debug.message;

    const IncrementalGraphSlamSnapshot before = slam.snapshot();
    ASSERT_TRUE(before.success);
    ASSERT_EQ(before.poses.size(), 1U);

    const IncrementalGraphSlamResult rejected = slam.process_frame(
        make_frame(9U, 100, transforms::Pose2D{5.0, 0.0, 0.0}));
    EXPECT_FALSE(rejected.debug.frame_accepted);
    EXPECT_FALSE(rejected.current_pose.has_value());
    EXPECT_EQ(rejected.debug.cumulative.pose_count, 1U);

    const IncrementalGraphSlamSnapshot after = slam.snapshot();
    ASSERT_TRUE(after.success);
    EXPECT_EQ(after.poses.size(), before.poses.size());
    EXPECT_EQ(after.landmarks.size(), before.landmarks.size());
}

TEST(IncrementalGraphSlamTest, RejectedFirstFrameDoesNotEstablishReferencePose)
{
    IncrementalGraphSlam slam(make_test_params());

    const IncrementalGraphSlamResult rejected = slam.process_frame(make_frame(
        0U, 100,
        transforms::Pose2D{std::numeric_limits<double>::quiet_NaN(), 0.0,
                           0.0}));
    EXPECT_FALSE(rejected.debug.frame_accepted);

    const IncrementalGraphSlamResult accepted = slam.process_frame(
        make_frame(1U, 200, transforms::Pose2D{10.0, 0.0, 0.0}));
    ASSERT_TRUE(accepted.debug.update_success) << accepted.debug.message;
    ASSERT_TRUE(accepted.current_pose.has_value());
    expect_pose_near(accepted.current_pose->initial_pose_map_from_base,
                     transforms::Pose2D{});
}

TEST(IncrementalGraphSlamTest, ZeroObservationFramesExtendPoseChain)
{
    IncrementalGraphSlam slam(make_test_params());

    ASSERT_TRUE(slam.process_frame(make_frame(0U, 100, transforms::Pose2D{}))
                    .debug.update_success);
    const IncrementalGraphSlamResult second = slam.process_frame(
        make_frame(1U, 200, transforms::Pose2D{1.0, 0.0, 0.0}));

    ASSERT_TRUE(second.debug.update_success) << second.debug.message;
    EXPECT_EQ(second.debug.update.observations_received, 0U);
    EXPECT_EQ(second.debug.cumulative.pose_count, 2U);
    EXPECT_EQ(second.debug.cumulative.between_factor_count, 1U);
}

}  // namespace
}  // namespace slam::backend
