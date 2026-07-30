#include <gtest/gtest.h>

#include <cstdint>
#include <utility>
#include <vector>

#include "backend/IncrementalGraphSlam.hpp"
#include "frontend/SlamFrontend.hpp"

namespace slam
{
namespace
{

frontend::SlamFrontendParams make_frontend_params()
{
    frontend::SlamFrontendParams params;
    params.optimized_association_gate_m = 1.0;
    params.local_track_association_gate_m = 0.5;
    params.minimum_observations_to_confirm = 2U;
    params.tentative_track_max_age_ns = 10'000'000'000LL;
    params.pending_track_max_age_ns = 10'000'000'000LL;
    params.minimum_detection_confidence = 0.5;
    return params;
}

backend::IncrementalGraphSlamParams make_backend_params()
{
    backend::IncrementalGraphSlamParams params;
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

ConeDetection make_detection(const double x_base_m, const double y_base_m = 0.0)
{
    return ConeDetection{transforms::Point2D{x_base_m, y_base_m}, 1.0};
}

ConeFrame make_cone_frame(const std::int64_t timestamp_ns,
                          std::vector<ConeDetection> detections)
{
    ConeFrame frame;
    frame.timestamp_ns = timestamp_ns;
    frame.pose_odom_from_base = transforms::Pose2D{};
    frame.detections = std::move(detections);
    return frame;
}

LandmarkFrame make_landmark_frame(const FrontendResult& result,
                                  const std::uint64_t frame_index)
{
    LandmarkFrame frame;
    frame.frame_index = frame_index;
    frame.timestamp_ns = result.timestamp_ns;
    frame.recorded_pose_odom_from_base = result.pose_odom_from_base;
    frame.observations = result.landmark_observations;
    return frame;
}

MapState make_map_state(const backend::IncrementalGraphSlamSnapshot& snapshot,
                        const std::uint64_t sequence,
                        const std::int64_t timestamp_ns)
{
    MapState state;
    state.sequence = sequence;
    state.timestamp_ns = timestamp_ns;
    state.pose_map_from_odom = *snapshot.latest_pose_map_from_odom;

    state.landmarks.reserve(snapshot.landmarks.size());
    for (const LandmarkEstimate& landmark : snapshot.landmarks)
    {
        state.landmarks.push_back(
            MapLandmark{landmark.landmark_id, landmark.optimized_position_map});
    }

    return state;
}

TEST(SlamEndToEndTest, FrontendBackendFeedbackAssociatesOptimizedMapLandmark)
{
    frontend::SlamFrontend frontend(make_frontend_params());
    backend::IncrementalGraphSlam backend(make_backend_params());

    const FrontendResult tentative = frontend.process_frame(
        make_cone_frame(100, {make_detection(5.0, 1.0)}));
    ASSERT_TRUE(tentative.frame_accepted) << tentative.message;
    EXPECT_TRUE(tentative.landmark_observations.empty());
    EXPECT_EQ(tentative.debug.tentative_tracks_created, 1U);

    const FrontendResult promoted = frontend.process_frame(
        make_cone_frame(200, {make_detection(5.1, 1.0)}));
    ASSERT_TRUE(promoted.frame_accepted) << promoted.message;
    ASSERT_EQ(promoted.landmark_observations.size(), 1U);
    EXPECT_EQ(promoted.landmark_observations.front().association,
              LandmarkAssociation::NewLandmark);

    const backend::IncrementalGraphSlamResult backend_result =
        backend.process_frame(make_landmark_frame(promoted, 0U));
    ASSERT_TRUE(backend_result.debug.update_success)
        << backend_result.debug.message;

    const backend::IncrementalGraphSlamSnapshot snapshot = backend.snapshot();
    ASSERT_TRUE(snapshot.success) << snapshot.message;
    ASSERT_TRUE(snapshot.latest_pose_map_from_odom.has_value());
    ASSERT_EQ(snapshot.landmarks.size(), 1U);

    const frontend::MapStateUpdateResult update =
        frontend.update_map_state(make_map_state(snapshot, 0U, 200));
    ASSERT_TRUE(update.accepted) << update.message;
    EXPECT_EQ(update.pending_tracks_resolved, 1U);

    const FrontendResult optimized = frontend.process_frame(
        make_cone_frame(300, {make_detection(5.05, 1.0)}));
    ASSERT_TRUE(optimized.frame_accepted) << optimized.message;
    ASSERT_EQ(optimized.landmark_observations.size(), 1U);
    EXPECT_EQ(optimized.landmark_observations.front().landmark_id,
              snapshot.landmarks.front().landmark_id);
    EXPECT_EQ(optimized.landmark_observations.front().association,
              LandmarkAssociation::ExistingMapLandmark);
    EXPECT_EQ(optimized.debug.optimized_associations, 1U);
}

}  // namespace
}  // namespace slam
