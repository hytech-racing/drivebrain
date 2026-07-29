#include <gtest/gtest.h>

#include <DriverlessEstimatorRunner.hpp>
#include <LatestEstimate.hpp>
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

DriverlessEstimatorRunner make_runner()
{
    const estimation::EkfParams ekf_params{0.1, 0.1, 0.5, 0.1, 0.1, 0.5};
    const estimation::GssSensorConfig gss_config{0.1, 0.1};
    auto transform_buffer =
        std::make_shared<transforms::TransformBuffer>(1000000);
    transform_buffer->set_base_to_gss(transforms::Pose2D{1.0, 0.25, 0.0});

    return DriverlessEstimatorRunner{
        std::make_shared<estimation::LatestEstimate>(), transform_buffer,
        ekf_params, gss_config, false};
}

estimation::ImuMeasurement make_imu(std::uint64_t timestamp_ns)
{
    estimation::ImuMeasurement measurement;
    measurement.timestamp_ns = timestamp_ns;
    measurement.ax_vehicle_flu_mps2 = 1.0;
    measurement.ay_vehicle_flu_mps2 = 2.0;
    measurement.yaw_rate_vehicle_flu_radps = 3.0;
    return measurement;
}

estimation::GssMeasurement make_gss(std::uint64_t timestamp_ns)
{
    estimation::GssMeasurement measurement;
    measurement.timestamp_ns = timestamp_ns;
    measurement.vx_sensor_flu_mps = 4.0;
    measurement.vy_sensor_flu_mps = 5.0;
    return measurement;
}

bool wait_until_stats(DriverlessEstimatorRunner& runner,
                      const std::uint64_t expected_imu_processed,
                      const std::uint64_t expected_gss_processed)
{
    constexpr int kMaxAttempts = 100;

    for (int attempt = 0; attempt < kMaxAttempts; ++attempt)
    {
        const EstimatorQueueStats stats = runner.stats();
        if (stats.imu_processed == expected_imu_processed &&
            stats.gss_processed == expected_gss_processed)
        {
            return true;
        }

        std::this_thread::sleep_for(1ms);
    }

    return false;
}

TEST(DriverlessEstimatorRunnerTest, StartAndStopCleanly)
{
    DriverlessEstimatorRunner runner = make_runner();

    runner.start();
    runner.stop();

    const EstimatorQueueStats stats = runner.stats();

    EXPECT_EQ(stats.current_queue_depth, 0U);
    EXPECT_EQ(stats.imu_processed, 0U);
    EXPECT_EQ(stats.gss_processed, 0U);
}

TEST(DriverlessEstimatorRunnerTest, EnqueueImuProcessesImu)
{
    DriverlessEstimatorRunner runner = make_runner();
    runner.start();

    EXPECT_TRUE(runner.enqueue(make_imu(100)));

    EXPECT_TRUE(wait_until_stats(runner, 1, 0));
    runner.stop();

    const EstimatorQueueStats stats = runner.stats();
    EXPECT_EQ(stats.imu_enqueued, 1U);
    EXPECT_EQ(stats.imu_processed, 1U);
    EXPECT_EQ(stats.latest_imu_timestamp_ns, 100U);
}

TEST(DriverlessEstimatorRunnerTest, EnqueueGssProcessesGss)
{
    DriverlessEstimatorRunner runner = make_runner();
    runner.start();

    EXPECT_TRUE(runner.enqueue(make_gss(200)));

    EXPECT_TRUE(wait_until_stats(runner, 0, 1));
    runner.stop();

    const EstimatorQueueStats stats = runner.stats();
    EXPECT_EQ(stats.gss_enqueued, 1U);
    EXPECT_EQ(stats.gss_processed, 1U);
    EXPECT_EQ(stats.latest_gss_timestamp_ns, 200U);
}

TEST(DriverlessEstimatorRunnerTest, MultipleEventsPreserved)
{
    DriverlessEstimatorRunner runner = make_runner();

    EXPECT_TRUE(runner.enqueue(make_imu(100)));
    EXPECT_TRUE(runner.enqueue(make_gss(200)));
    EXPECT_TRUE(runner.enqueue(make_imu(300)));
    EXPECT_TRUE(runner.enqueue(make_gss(400)));

    runner.start();
    EXPECT_TRUE(wait_until_stats(runner, 2, 2));
    runner.stop();

    const EstimatorQueueStats stats = runner.stats();
    EXPECT_EQ(stats.imu_enqueued, 2U);
    EXPECT_EQ(stats.gss_enqueued, 2U);
    EXPECT_EQ(stats.current_queue_depth, 0U);
    EXPECT_EQ(stats.maximum_queue_depth, 4U);
}

TEST(DriverlessEstimatorRunnerTest, MultipleProducerThreadsAreSafe)
{
    constexpr int kProducerCount = 4;
    constexpr int kEventsPerProducer = 50;

    DriverlessEstimatorRunner runner = make_runner();

    std::vector<std::thread> producers;
    producers.reserve(kProducerCount);

    for (int producer = 0; producer < kProducerCount; ++producer)
    {
        producers.emplace_back(
            [&runner]()
            {
                for (int event = 0; event < kEventsPerProducer; ++event)
                {
                    EXPECT_TRUE(runner.enqueue(make_imu(100)));
                }
            });
    }

    for (std::thread& producer : producers)
    {
        producer.join();
    }

    runner.start();
    EXPECT_TRUE(
        wait_until_stats(runner, kProducerCount * kEventsPerProducer, 0));
    runner.stop();

    const EstimatorQueueStats stats = runner.stats();
    EXPECT_EQ(stats.imu_enqueued, kProducerCount * kEventsPerProducer);
    EXPECT_EQ(stats.imu_processed, kProducerCount * kEventsPerProducer);
    EXPECT_EQ(stats.queue_drops, 0U);
}

TEST(DriverlessEstimatorRunnerTest, QueueOverflowReported)
{
    DriverlessEstimatorRunner runner = make_runner();

    for (std::uint64_t i = 0; i < 4096; ++i)
    {
        ASSERT_TRUE(runner.enqueue(make_imu(i + 1)));
    }

    EXPECT_FALSE(runner.enqueue(make_imu(4097)));

    const EstimatorQueueStats stats = runner.stats();
    EXPECT_EQ(stats.queue_drops, 1U);
    EXPECT_EQ(stats.current_queue_depth, 4096U);
}

TEST(DriverlessEstimatorRunnerTest, OldTimestampRejected)
{
    DriverlessEstimatorRunner runner = make_runner();

    EXPECT_TRUE(runner.enqueue(make_imu(200)));
    EXPECT_FALSE(runner.enqueue(make_imu(100)));

    const EstimatorQueueStats stats = runner.stats();
    EXPECT_EQ(stats.imu_enqueued, 1U);
    EXPECT_EQ(stats.out_of_order_measurements, 1U);
    EXPECT_EQ(stats.latest_imu_timestamp_ns, 200U);
}

TEST(DriverlessEstimatorRunnerTest, NoProcessingOccursAfterStop)
{
    DriverlessEstimatorRunner runner = make_runner();
    runner.start();
    runner.stop();

    EXPECT_TRUE(runner.enqueue(make_imu(100)));
    std::this_thread::sleep_for(10ms);

    const EstimatorQueueStats stats = runner.stats();
    EXPECT_EQ(stats.imu_processed, 0U);
    EXPECT_EQ(stats.current_queue_depth, 1U);
}

TEST(DriverlessEstimatorRunnerTest, StopDrainsPendingEvents)
{
    DriverlessEstimatorRunner runner = make_runner();

    EXPECT_TRUE(runner.enqueue(make_imu(100)));
    EXPECT_TRUE(runner.enqueue(make_gss(200)));
    EXPECT_TRUE(runner.enqueue(make_imu(300)));

    runner.start();
    runner.stop();

    const EstimatorQueueStats stats = runner.stats();
    EXPECT_EQ(stats.imu_processed, 2U);
    EXPECT_EQ(stats.gss_processed, 1U);
    EXPECT_EQ(stats.current_queue_depth, 0U);
}

TEST(DriverlessEstimatorRunnerTest, StaleCrossSensorMeasurementIsDropped)
{
    DriverlessEstimatorRunner runner = make_runner();

    EXPECT_TRUE(runner.enqueue(make_imu(100)));
    EXPECT_TRUE(runner.enqueue(make_imu(200)));
    EXPECT_TRUE(runner.enqueue(make_gss(150)));

    runner.start();
    EXPECT_TRUE(wait_until_stats(runner, 2, 0));
    runner.stop();

    const EstimatorQueueStats stats = runner.stats();
    EXPECT_EQ(stats.imu_processed, 2U);
    EXPECT_EQ(stats.gss_processed, 0U);
    EXPECT_EQ(stats.out_of_order_measurements, 1U);
    EXPECT_EQ(stats.current_queue_depth, 0U);
}

}  // namespace
}  // namespace runtime
