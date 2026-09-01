#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <thread>

#include "backend/IncrementalGraphSlam.hpp"
#include "backend/IncrementalGraphSlamTypes.hpp"
#include "common/LatestMapState.hpp"
namespace runtime
{

struct LandmarkFrameQueueStats
{
    std::size_t landmark_frames_enqueued{};

    std::size_t landmark_frames_processed{};

    std::size_t landmark_frames_processing_failed{};

    std::size_t frames_rejected_not_running{};

    std::size_t nonincreasing_landmark_frames{};
    std::size_t queue_overflows{};

    std::size_t current_queue_depth{};
    std::size_t maximum_queue_depth{};

    std::int64_t latest_landmark_frame_timestamp_ns{};
};

class SlamBackendRunner
{
   public:
    SlamBackendRunner(std::shared_ptr<slam::LatestMapState> latest_map_state,
                      const slam::backend::IncrementalGraphSlamParams& params,
                      bool publish_full_telemetry);

    ~SlamBackendRunner();

    void start();

    void stop();

    bool enqueue(slam::LandmarkFrame&& landmark_frame);

    LandmarkFrameQueueStats landmark_frame_queue_stats() const;

   private:
    void _run();

    [[nodiscard]] bool _process(const slam::LandmarkFrame& frame);

    void _publish_runner_debug(std::int64_t timestamp_ns) const;

   private:
    static constexpr std::size_t kMaximumQueueSize = 100;

   private:
    std::shared_ptr<slam::LatestMapState> _latest_map_state;

    bool _publish_full_telemetry;

    std::thread _thread;
    std::atomic<bool> _running{false};

    slam::backend::IncrementalGraphSlam _graph_slam;

    std::uint64_t _next_map_sequence{0U};

   private:
    mutable std::mutex _queue_mutex;
    std::condition_variable _queue_cv;
    std::deque<slam::LandmarkFrame> _landmark_frame_queue;

   private:
    mutable std::mutex _stats_mutex;
    LandmarkFrameQueueStats _stats;
};

}  // namespace runtime
