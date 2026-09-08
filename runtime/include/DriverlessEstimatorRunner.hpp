
#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "EkfEstimator.hpp"
#include "EstimatorMeasurements.hpp"
#include "LatestEstimate.hpp"
#include "TransformBuffer.hpp"

namespace runtime
{

struct EstimatorQueueStats
{
    std::uint64_t imu_enqueued{};
    std::uint64_t gss_enqueued{};

    std::uint64_t imu_processed{};
    std::uint64_t gss_processed{};

    std::uint64_t invalid_measurements{};
    std::uint64_t out_of_order_measurements{};
    std::uint64_t queue_drops{};

    std::size_t current_queue_depth{};
    std::size_t maximum_queue_depth{};

    std::uint64_t latest_imu_timestamp_ns{};
    std::uint64_t latest_gss_timestamp_ns{};
};

class DriverlessEstimatorRunner
{
   public:
    DriverlessEstimatorRunner(
        std::shared_ptr<estimation::LatestEstimate> latest_estimate,
        std::shared_ptr<transforms::TransformBuffer> transform_buffer,
        const estimation::EkfParams& ekf_params,
        const estimation::GssSensorConfig& gss_sensor_config,
        bool publish_full_telemetry = true);

    ~DriverlessEstimatorRunner();

    void start();

    void stop();

    bool enqueue(estimation::ImuMeasurement measurement);
    bool enqueue(estimation::GssMeasurement measurement);

    EstimatorQueueStats stats() const;

   private:
    bool _enqueue_event(estimation::EstimatorEvent event);

    void _run();

    [[nodiscard]] bool _process_event(const estimation::EstimatorEvent& event);

    void _publish_estimate(const estimation::StateEstimate& estimate,
                           bool publish_full_telemetry);

    void _publish_odom_trail(const estimation::StateEstimate& estimate);

   private:
    static constexpr std::size_t kMaximumQueueSize = 4096;
    static constexpr std::size_t kMaximumOdomTrailPoints =
        10000;  // a little less than 4 laps with current settings before points
                // get popped
    static constexpr std::uint64_t kOdomTrailPublishPeriodNs = 100'000'000ULL;
    static constexpr std::uint64_t kStaticTransformPublishPeriodNs =
        1'000'000'000ULL;
    static constexpr double kOdomTrailPointDistanceThresholdM = 0.10;
    static constexpr double kOdomTrailPointYawThresholdRad = 0.05;

    std::shared_ptr<estimation::LatestEstimate> _latest_estimate;
    std::shared_ptr<transforms::TransformBuffer> _transform_buffer;
    bool _publish_full_telemetry;

    mutable std::mutex _queue_mutex;
    std::condition_variable _queue_cv;
    std::deque<estimation::EstimatorEvent> _queue;

    std::thread _thread;
    std::atomic<bool> _running{false};

    estimation::EkfEstimator _ekf;

    mutable std::mutex _stats_mutex;
    EstimatorQueueStats _stats;

    std::optional<std::uint64_t> _previous_imu_stamp_ns;
    std::optional<std::uint64_t> _previous_gss_stamp_ns;

    std::deque<std::array<float, 3>> _odom_trail;
    std::optional<std::uint64_t> _last_odom_trail_publish_stamp_ns;
    std::optional<std::uint64_t> _last_static_transform_publish_stamp_ns;
    std::optional<double> _last_odom_trail_point_yaw_rad;
};

}  // namespace runtime
