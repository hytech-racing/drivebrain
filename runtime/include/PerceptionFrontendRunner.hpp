#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "LidarProcessor.hpp"
#include "PointCloudTypes.hpp"
#include "TransformBuffer.hpp"

namespace runtime
{

struct PointCloudQueueStats
{
    std::size_t point_clouds_enqueued{};

    std::size_t point_clouds_processed{};

    std::size_t out_of_order_point_clouds{};
    std::size_t queue_trims{};

    std::size_t current_queue_depth{};
    std::size_t maximum_queue_depth{};

    std::int64_t latest_point_cloud_timestamp_ns{};
};

class PerceptionFrontendRunner
{
   public:
    PerceptionFrontendRunner(
        std::shared_ptr<transforms::TransformBuffer> transform_buffer,
        perception::LidarProcessorParams lidar_processor_params = {},
        bool publish_full_telemetry = true);

    ~PerceptionFrontendRunner();

    void start();

    void stop();

    bool enqueue(perception::StampedPointCloud&& stamped_point_cloud);

    PointCloudQueueStats point_cloud_queue_stats() const;

   private:
    void _run();

    [[nodiscard]] bool _process_point_cloud(
        const perception::StampedPointCloud& stamped_point_cloud);

   private:
    static constexpr std::size_t kMaximumQueueSize = 3;

   private:
    std::shared_ptr<transforms::TransformBuffer> _transform_buffer;
    perception::LidarProcessorParams _lidar_processor_params;

    bool _publish_full_telemetry;

    std::thread _point_cloud_processor_thread;
    std::atomic<bool> _point_cloud_processor_running{false};

    std::optional<std::int64_t> _previous_point_cloud_stamp_ns;

    perception::LidarProcessor _lidar_processor;

   private:
    mutable std::mutex _point_cloud_queue_mutex;
    std::condition_variable _point_cloud_queue_cv;
    std::deque<perception::StampedPointCloud> _point_cloud_queue;

   private:
    mutable std::mutex _point_cloud_queue_stats_mutex;
    PointCloudQueueStats _point_cloud_queue_stats;
};

}  // namespace runtime
