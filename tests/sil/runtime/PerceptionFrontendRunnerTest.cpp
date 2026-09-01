#include <gtest/gtest.h>

#include <PerceptionFrontendRunner.hpp>
#include <TransformBuffer.hpp>
#include <chrono>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

namespace runtime
{
namespace
{

using namespace std::chrono_literals;

PerceptionFrontendRunner make_runner()
{
    auto transform_buffer =
        std::make_shared<transforms::TransformBuffer>(1000000);
    transform_buffer->set_T_base_lidar(transforms::Pose2D{});
    transform_buffer->insert_T_odom_base(1, transforms::Pose2D{});
    transform_buffer->insert_T_odom_base(1000000, transforms::Pose2D{});

    auto latest_map_state = std::make_shared<slam::LatestMapState>();

    return PerceptionFrontendRunner{
        transform_buffer,
        latest_map_state,
        [](slam::LandmarkFrame) { return true; },
        perception::LidarProcessorParams{},
        slam::frontend::SlamFrontendParams{1.0, 1.0, 5U, 3'000'000'000LL,
                                           1'000'000'000LL, 0.5},
        false};
}

perception::StampedPointCloud make_point_cloud(std::uint64_t timestamp_ns)
{
    perception::StampedPointCloud point_cloud;
    point_cloud.timestamp_ns = timestamp_ns;
    point_cloud.points.push_back(perception::PointXYZI{1.0F, 2.0F, 3.0F, 4.0F});
    return point_cloud;
}

bool wait_until_stats(PerceptionFrontendRunner& runner,
                      const std::uint64_t expected_point_clouds_processed)
{
    constexpr int kMaxAttempts = 100;

    for (int attempt = 0; attempt < kMaxAttempts; ++attempt)
    {
        const PointCloudQueueStats stats = runner.point_cloud_queue_stats();
        if (stats.point_clouds_processed == expected_point_clouds_processed)
        {
            return true;
        }

        std::this_thread::sleep_for(1ms);
    }

    return false;
}

TEST(PerceptionFrontendRunnerTest, StartAndStopCleanly)
{
    PerceptionFrontendRunner runner = make_runner();

    runner.start();
    runner.stop();

    const PointCloudQueueStats stats = runner.point_cloud_queue_stats();

    EXPECT_EQ(stats.current_queue_depth, 0U);
    EXPECT_EQ(stats.point_clouds_processed, 0U);
}

TEST(PerceptionFrontendRunnerTest, EnqueuePointCloudProcessesPointCloud)
{
    PerceptionFrontendRunner runner = make_runner();
    runner.start();

    EXPECT_TRUE(runner.enqueue(make_point_cloud(100)));

    EXPECT_TRUE(wait_until_stats(runner, 1));
    runner.stop();

    const PointCloudQueueStats stats = runner.point_cloud_queue_stats();
    EXPECT_EQ(stats.point_clouds_enqueued, 1U);
    EXPECT_EQ(stats.point_clouds_processed, 1U);
    EXPECT_EQ(stats.latest_point_cloud_timestamp_ns, 100U);
}

TEST(PerceptionFrontendRunnerTest, MultiplePointCloudsPreserved)
{
    PerceptionFrontendRunner runner = make_runner();

    EXPECT_TRUE(runner.enqueue(make_point_cloud(100)));
    EXPECT_TRUE(runner.enqueue(make_point_cloud(200)));
    EXPECT_TRUE(runner.enqueue(make_point_cloud(300)));

    runner.start();
    EXPECT_TRUE(wait_until_stats(runner, 3));
    runner.stop();

    const PointCloudQueueStats stats = runner.point_cloud_queue_stats();
    EXPECT_EQ(stats.point_clouds_enqueued, 3U);
    EXPECT_EQ(stats.point_clouds_processed, 3U);
    EXPECT_EQ(stats.current_queue_depth, 0U);
    EXPECT_EQ(stats.maximum_queue_depth, 3U);
}

TEST(PerceptionFrontendRunnerTest, MultipleProducerThreadsAreSafe)
{
    constexpr int kProducerCount = 4;
    constexpr int kPointCloudsPerProducer = 50;

    PerceptionFrontendRunner runner = make_runner();

    std::vector<std::thread> producers;
    producers.reserve(kProducerCount);

    for (int producer = 0; producer < kProducerCount; ++producer)
    {
        producers.emplace_back(
            [&runner]()
            {
                for (int point_cloud = 0; point_cloud < kPointCloudsPerProducer;
                     ++point_cloud)
                {
                    runner.enqueue(make_point_cloud(100));
                }
            });
    }

    for (std::thread& producer : producers)
    {
        producer.join();
    }

    runner.start();
    EXPECT_TRUE(wait_until_stats(runner, 1));
    runner.stop();

    const PointCloudQueueStats stats = runner.point_cloud_queue_stats();
    EXPECT_EQ(stats.point_clouds_enqueued, 1U);
    EXPECT_EQ(stats.point_clouds_processed, 1U);
    EXPECT_EQ(stats.nonincreasing_point_clouds,
              kProducerCount * kPointCloudsPerProducer - 1);
    EXPECT_EQ(stats.queue_trims, 0U);
}

TEST(PerceptionFrontendRunnerTest, QueueOverflowTrimsOldestPointClouds)
{
    PerceptionFrontendRunner runner = make_runner();

    EXPECT_TRUE(runner.enqueue(make_point_cloud(100)));
    EXPECT_TRUE(runner.enqueue(make_point_cloud(200)));
    EXPECT_TRUE(runner.enqueue(make_point_cloud(300)));
    EXPECT_TRUE(runner.enqueue(make_point_cloud(400)));

    const PointCloudQueueStats stats = runner.point_cloud_queue_stats();
    EXPECT_EQ(stats.point_clouds_enqueued, 4U);
    EXPECT_EQ(stats.queue_trims, 1U);
    EXPECT_EQ(stats.current_queue_depth, 3U);
    EXPECT_EQ(stats.maximum_queue_depth, 3U);
    EXPECT_EQ(stats.latest_point_cloud_timestamp_ns, 400U);
}

TEST(PerceptionFrontendRunnerTest, OldTimestampRejected)
{
    PerceptionFrontendRunner runner = make_runner();

    EXPECT_TRUE(runner.enqueue(make_point_cloud(200)));
    EXPECT_FALSE(runner.enqueue(make_point_cloud(100)));

    const PointCloudQueueStats stats = runner.point_cloud_queue_stats();
    EXPECT_EQ(stats.point_clouds_enqueued, 1U);
    EXPECT_EQ(stats.nonincreasing_point_clouds, 1U);
    EXPECT_EQ(stats.latest_point_cloud_timestamp_ns, 200U);
}

TEST(PerceptionFrontendRunnerTest, NoProcessingOccursAfterStop)
{
    PerceptionFrontendRunner runner = make_runner();
    runner.start();
    runner.stop();

    EXPECT_TRUE(runner.enqueue(make_point_cloud(100)));
    std::this_thread::sleep_for(10ms);

    const PointCloudQueueStats stats = runner.point_cloud_queue_stats();
    EXPECT_EQ(stats.point_clouds_processed, 0U);
    EXPECT_EQ(stats.current_queue_depth, 1U);
}

TEST(PerceptionFrontendRunnerTest, NonLidarFrameCloudIsNotProcessed)
{
    PerceptionFrontendRunner runner = make_runner();
    perception::StampedPointCloud point_cloud = make_point_cloud(100);
    point_cloud.frame = transforms::FrameId::Map;

    runner.start();

    EXPECT_TRUE(runner.enqueue(std::move(point_cloud)));
    std::this_thread::sleep_for(10ms);
    runner.stop();

    const PointCloudQueueStats stats = runner.point_cloud_queue_stats();
    EXPECT_EQ(stats.point_clouds_enqueued, 1U);
    EXPECT_EQ(stats.point_clouds_processed, 0U);
    EXPECT_EQ(stats.current_queue_depth, 0U);
}

TEST(PerceptionFrontendRunnerTest, StopDrainsPendingPointClouds)
{
    PerceptionFrontendRunner runner = make_runner();

    EXPECT_TRUE(runner.enqueue(make_point_cloud(100)));
    EXPECT_TRUE(runner.enqueue(make_point_cloud(200)));
    EXPECT_TRUE(runner.enqueue(make_point_cloud(300)));

    runner.start();
    runner.stop();

    const PointCloudQueueStats stats = runner.point_cloud_queue_stats();
    EXPECT_EQ(stats.point_clouds_processed, 3U);
    EXPECT_EQ(stats.current_queue_depth, 0U);
}

}  // namespace
}  // namespace runtime
