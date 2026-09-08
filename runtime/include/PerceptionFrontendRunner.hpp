#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

#include "LidarProcessor.hpp"
#include "PointCloudTypes.hpp"
#include "TransformBuffer.hpp"
#include "common/LatestMapState.hpp"
#include "common/LatestPlannerMap.hpp"
#include "frontend/SlamFrontend.hpp"

namespace runtime
{

struct PointCloudQueueStats
{
    std::size_t point_clouds_enqueued{};

    std::size_t point_clouds_processed{};

    std::size_t point_clouds_processing_failed{};

    std::size_t nonincreasing_point_clouds{};
    std::size_t queue_trims{};

    std::size_t current_queue_depth{};
    std::size_t maximum_queue_depth{};

    std::int64_t latest_point_cloud_timestamp_ns{};
};

struct MapStateUpdateStats
{
    std::size_t update_success{};
    std::size_t update_failed{};

    std::string message{};

    std::size_t landmark_count{};
    std::size_t pending_tracks_resolved{};
};

class PerceptionFrontendRunner
{
   public:
    // slam backend callback
    using LandmarkFrameHandler = std::function<bool(slam::LandmarkFrame)>;

    PerceptionFrontendRunner(
        std::shared_ptr<transforms::TransformBuffer> transform_buffer,
        std::shared_ptr<slam::LatestMapState> latest_map_state,
        LandmarkFrameHandler landmark_frame_handler,
        perception::LidarProcessorParams lidar_processor_params = {},
        slam::frontend::SlamFrontendParams slam_frontend_params =
            {1.0, 1.0, 5U, 3'000'000'000LL, 1'000'000'000LL, 0.5},
        bool publish_full_telemetry = true,
        std::shared_ptr<slam::LatestPlannerMap> latest_planner_map = nullptr);

    ~PerceptionFrontendRunner();

    void start();

    void stop();

    bool enqueue(perception::StampedPointCloud&& stamped_point_cloud);

    PointCloudQueueStats point_cloud_queue_stats() const;

   private:
    void _run();

    [[nodiscard]] bool _process_point_cloud(
        const perception::StampedPointCloud& stamped_point_cloud);

    void _apply_latest_map_state();

    void _publish_slam_frontend_debug(
        const slam::FrontendResult& frontend_result) const;

    void _publish_runner_debug(std::int64_t timestamp_ns) const;

   private:
    static constexpr std::size_t kMaximumQueueSize = 3;

   private:
    std::shared_ptr<transforms::TransformBuffer> _transform_buffer;
    std::shared_ptr<slam::LatestMapState> _latest_map_state;
    std::shared_ptr<slam::LatestPlannerMap> _latest_planner_map;

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

   private:
    LandmarkFrameHandler _landmark_frame_handler;
    std::uint64_t _next_landmark_frame_index{};
    std::uint64_t _next_planner_map_sequence{};
    slam::frontend::SlamFrontendParams _frontend_params;
    slam::frontend::SlamFrontend _slam_frontend;
    std::optional<std::uint64_t> _last_consumed_map_sequence;
    MapStateUpdateStats _map_state_update_stats;
};

}  // namespace runtime
