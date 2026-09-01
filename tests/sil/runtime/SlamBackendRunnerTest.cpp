#include <gtest/gtest.h>

#include <SlamBackendRunner.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

namespace runtime
{
namespace
{

using namespace std::chrono_literals;

slam::backend::IncrementalGraphSlamParams make_test_params()
{
    slam::backend::IncrementalGraphSlamParams params;
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

slam::LandmarkObservation make_observation(const std::uint64_t landmark_id,
                                           const double x_base_m,
                                           const double y_base_m)
{
    slam::LandmarkObservation observation;
    observation.landmark_id = landmark_id;
    observation.measurement_base_m = transforms::Point2D{x_base_m, y_base_m};
    observation.association = slam::LandmarkAssociation::NewLandmark;
    return observation;
}

slam::LandmarkFrame make_frame(
    const std::uint64_t frame_index, const std::int64_t timestamp_ns,
    std::vector<slam::LandmarkObservation> observations = {})
{
    slam::LandmarkFrame frame;
    frame.frame_index = frame_index;
    frame.timestamp_ns = timestamp_ns;
    frame.recorded_pose_odom_from_base = transforms::Pose2D{};
    frame.observations = std::move(observations);
    return frame;
}

SlamBackendRunner make_runner(std::shared_ptr<slam::LatestMapState> map_state)
{
    return SlamBackendRunner{map_state, make_test_params(), false};
}

bool wait_until_processed(SlamBackendRunner& runner,
                          const std::size_t expected_processed)
{
    constexpr int kMaxAttempts = 100;

    for (int attempt = 0; attempt < kMaxAttempts; ++attempt)
    {
        const LandmarkFrameQueueStats stats =
            runner.landmark_frame_queue_stats();
        if (stats.landmark_frames_processed == expected_processed)
        {
            return true;
        }

        std::this_thread::sleep_for(1ms);
    }

    return false;
}

TEST(SlamBackendRunnerTest, NullMapStateThrows)
{
    EXPECT_THROW(SlamBackendRunner(nullptr, make_test_params(), false),
                 std::invalid_argument);
}

TEST(SlamBackendRunnerTest, StartAndStopCleanly)
{
    SlamBackendRunner runner =
        make_runner(std::make_shared<slam::LatestMapState>());

    runner.start();
    runner.stop();

    const LandmarkFrameQueueStats stats = runner.landmark_frame_queue_stats();
    EXPECT_EQ(stats.current_queue_depth, 0U);
    EXPECT_EQ(stats.landmark_frames_processed, 0U);
}

TEST(SlamBackendRunnerTest, RejectsFramesBeforeStart)
{
    SlamBackendRunner runner =
        make_runner(std::make_shared<slam::LatestMapState>());

    EXPECT_FALSE(runner.enqueue(make_frame(0U, 100)));

    const LandmarkFrameQueueStats stats = runner.landmark_frame_queue_stats();
    EXPECT_EQ(stats.frames_rejected_not_running, 1U);
    EXPECT_EQ(stats.landmark_frames_enqueued, 0U);
}

TEST(SlamBackendRunnerTest, EnqueuedFramePublishesLatestMapState)
{
    auto latest_map_state = std::make_shared<slam::LatestMapState>();
    SlamBackendRunner runner = make_runner(latest_map_state);
    runner.start();

    EXPECT_TRUE(runner.enqueue(
        make_frame(0U, 100, {make_observation(42U, 5.0, 1.0)})));
    EXPECT_TRUE(wait_until_processed(runner, 1U));
    runner.stop();

    const LandmarkFrameQueueStats stats = runner.landmark_frame_queue_stats();
    EXPECT_EQ(stats.landmark_frames_enqueued, 1U);
    EXPECT_EQ(stats.landmark_frames_processed, 1U);
    EXPECT_EQ(stats.latest_landmark_frame_timestamp_ns, 100);

    const std::optional<slam::MapState> map_state = latest_map_state->latest();
    ASSERT_TRUE(map_state.has_value());
    EXPECT_EQ(map_state->sequence, 0U);
    EXPECT_EQ(map_state->timestamp_ns, 100);
    ASSERT_EQ(map_state->landmarks.size(), 1U);
    EXPECT_EQ(map_state->landmarks.front().landmark_id, 42U);
}

TEST(SlamBackendRunnerTest, RejectsNonIncreasingTimestamps)
{
    SlamBackendRunner runner =
        make_runner(std::make_shared<slam::LatestMapState>());
    runner.start();

    EXPECT_TRUE(runner.enqueue(make_frame(0U, 100)));
    EXPECT_FALSE(runner.enqueue(make_frame(1U, 100)));
    EXPECT_FALSE(runner.enqueue(make_frame(2U, 50)));

    EXPECT_TRUE(wait_until_processed(runner, 1U));
    runner.stop();

    const LandmarkFrameQueueStats stats = runner.landmark_frame_queue_stats();
    EXPECT_EQ(stats.landmark_frames_enqueued, 1U);
    EXPECT_EQ(stats.nonincreasing_landmark_frames, 2U);
}

TEST(SlamBackendRunnerTest, StopDrainsPendingFrames)
{
    SlamBackendRunner runner =
        make_runner(std::make_shared<slam::LatestMapState>());
    runner.start();

    EXPECT_TRUE(runner.enqueue(make_frame(0U, 100)));
    EXPECT_TRUE(runner.enqueue(make_frame(1U, 200)));
    EXPECT_TRUE(runner.enqueue(make_frame(2U, 300)));

    runner.stop();

    const LandmarkFrameQueueStats stats = runner.landmark_frame_queue_stats();
    EXPECT_EQ(stats.landmark_frames_enqueued, 3U);
    EXPECT_EQ(stats.landmark_frames_processed, 3U);
    EXPECT_EQ(stats.current_queue_depth, 0U);
}

}  // namespace
}  // namespace runtime
