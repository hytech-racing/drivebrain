#include "SlamBackendRunner.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>

#include "SlamDebugMessageAdapters.hpp"
#include "SlamVisualizationAdapters.hpp"
#include "Telemetry.hpp"
#include "dv_msgs.pb.h"

namespace runtime
{
namespace
{

std::uint32_t to_debug_count(const std::size_t count)
{
    return count > std::numeric_limits<std::uint32_t>::max()
               ? std::numeric_limits<std::uint32_t>::max()
               : static_cast<std::uint32_t>(count);
}

}  // namespace

SlamBackendRunner::SlamBackendRunner(
    std::shared_ptr<slam::LatestMapState> latest_map_state,
    const slam::backend::IncrementalGraphSlamParams& params,
    bool publish_full_telemetry)
    : _latest_map_state(std::move(latest_map_state)),
      _graph_slam(params),
      _publish_full_telemetry(publish_full_telemetry)
{
    if (!_latest_map_state)
    {
        throw std::invalid_argument(
            "SlamBackendRunner requires LatestMapState");
    }
}

SlamBackendRunner::~SlamBackendRunner() { stop(); }

void SlamBackendRunner::start()
{
    {
        std::scoped_lock lock(_queue_mutex);

        if (_running)
        {
            return;
        }

        _running = true;
    }

    _thread = std::thread(&SlamBackendRunner::_run, this);
}

void SlamBackendRunner::stop()
{
    if (!_running)
    {
        return;
    }

    {
        std::scoped_lock lock(_queue_mutex);
        _running = false;
    }

    _queue_cv.notify_one();

    if (_thread.joinable())
    {
        _thread.join();
    }
}

bool SlamBackendRunner::enqueue(slam::LandmarkFrame&& landmark_frame)
{
    {
        std::scoped_lock lock(_queue_mutex, _stats_mutex);

        if (!_running)
        {
            _stats.frames_rejected_not_running++;
            return false;
        }

        if (_stats.landmark_frames_enqueued > 0U &&
            landmark_frame.timestamp_ns <=
                _stats.latest_landmark_frame_timestamp_ns)
        {
            _stats.nonincreasing_landmark_frames++;
            return false;
        }

        if (_landmark_frame_queue.size() >= kMaximumQueueSize)
        {
            _stats.queue_overflows++;
            return false;
        }

        _stats.latest_landmark_frame_timestamp_ns = landmark_frame.timestamp_ns;
        _stats.landmark_frames_enqueued++;

        _landmark_frame_queue.push_back(std::move(landmark_frame));

        _stats.current_queue_depth = _landmark_frame_queue.size();
        _stats.maximum_queue_depth =
            std::max(_stats.maximum_queue_depth, _stats.current_queue_depth);
    }

    _queue_cv.notify_one();
    return true;
}

LandmarkFrameQueueStats SlamBackendRunner::landmark_frame_queue_stats() const
{
    std::scoped_lock lock(_queue_mutex, _stats_mutex);

    LandmarkFrameQueueStats stats = _stats;
    stats.current_queue_depth = _landmark_frame_queue.size();
    return stats;
}

void SlamBackendRunner::_run()
{
    while (true)
    {
        slam::LandmarkFrame landmark_frame;

        {
            std::unique_lock queue_lock(_queue_mutex);

            _queue_cv.wait(
                queue_lock, [this]()
                { return !_running || !_landmark_frame_queue.empty(); });

            if (!_running && _landmark_frame_queue.empty())
            {
                return;
            }

            landmark_frame = std::move(_landmark_frame_queue.front());
            _landmark_frame_queue.pop_front();

            {
                std::scoped_lock stats_lock(_stats_mutex);
                _stats.current_queue_depth = _landmark_frame_queue.size();
            }
        }

        const bool processed = _process(landmark_frame);

        {
            std::scoped_lock stats_lock(_stats_mutex);

            if (processed)
            {
                _stats.landmark_frames_processed++;
            }
            else
            {
                _stats.landmark_frames_processing_failed++;
            }
        }

        if (_publish_full_telemetry)
        {
            _publish_runner_debug(landmark_frame.timestamp_ns);
        }
    }
}

bool SlamBackendRunner::_process(const slam::LandmarkFrame& frame)
{
    const slam::backend::IncrementalGraphSlamResult result =
        _graph_slam.process_frame(frame);

    if (_publish_full_telemetry)
    {
        core::log("/slam/backend/debug",
                  adapters::to_incremental_graph_slam_debug(frame, result));
    }

    bool process_successful = result.debug.frame_accepted &&
                              !result.debug.core_failed &&
                              result.debug.update_success;

    if (!process_successful)
    {
        return false;
    }

    const auto snapshot = _graph_slam.snapshot();

    if (!snapshot.success || !snapshot.latest_pose_map_from_odom.has_value())
    {
        return false;
    }

    if (_publish_full_telemetry)
    {
        core::log("/slam/incremental/initial_path_map",
                  adapters::to_foxglove_slam_pose_markers(
                      snapshot.poses, false, frame.timestamp_ns,
                      "slam_initial_pose_markers"));
        core::log("/slam/incremental/optimized_path_map",
                  adapters::to_foxglove_slam_pose_markers(
                      snapshot.poses, true, frame.timestamp_ns,
                      "slam_optimized_pose_markers"));
        core::log("/slam/incremental/initial_landmark_map_markers",
                  adapters::to_foxglove_slam_landmark_markers(
                      snapshot.landmarks, false, frame.timestamp_ns,
                      "slam_initial_landmarks"));
        core::log("/slam/incremental/optimized_landmark_map_markers",
                  adapters::to_foxglove_slam_landmark_markers(
                      snapshot.landmarks, true, frame.timestamp_ns,
                      "slam_optimized_landmarks"));
        core::log("/slam/incremental/map_landmark_text",
                  adapters::to_foxglove_slam_landmark_text(snapshot.landmarks,
                                                           frame.timestamp_ns));
        core::log("/tf",
                  adapters::to_foxglove_map_odom_transform(
                      *snapshot.latest_pose_map_from_odom, frame.timestamp_ns));
    }

    slam::MapState map_state;
    map_state.sequence = _next_map_sequence++;
    map_state.timestamp_ns = frame.timestamp_ns;
    map_state.pose_map_from_odom = *snapshot.latest_pose_map_from_odom;

    map_state.landmarks.reserve(snapshot.landmarks.size());

    for (const auto& landmark : snapshot.landmarks)
    {
        map_state.landmarks.push_back(slam::MapLandmark{
            landmark.landmark_id, landmark.optimized_position_map});
    }

    _latest_map_state->store(std::move(map_state));

    return true;
}

void SlamBackendRunner::_publish_runner_debug(
    const std::int64_t timestamp_ns) const
{
    auto message = std::make_shared<dv_msgs::SlamBackendRunnerDebug>();
    message->set_timestamp_ns(timestamp_ns);

    const LandmarkFrameQueueStats stats = landmark_frame_queue_stats();
    message->set_landmark_frames_enqueued(
        to_debug_count(stats.landmark_frames_enqueued));
    message->set_landmark_frames_processed(
        to_debug_count(stats.landmark_frames_processed));
    message->set_landmark_frames_processing_failed(
        to_debug_count(stats.landmark_frames_processing_failed));
    message->set_frames_rejected_not_running(
        to_debug_count(stats.frames_rejected_not_running));
    message->set_nonincreasing_landmark_frames(
        to_debug_count(stats.nonincreasing_landmark_frames));
    message->set_queue_overflows(to_debug_count(stats.queue_overflows));
    message->set_current_queue_depth(to_debug_count(stats.current_queue_depth));
    message->set_maximum_queue_depth(to_debug_count(stats.maximum_queue_depth));
    message->set_latest_landmark_frame_timestamp_ns(
        stats.latest_landmark_frame_timestamp_ns);

    core::log("/slam/backend/runner_debug", message);
}

}  // namespace runtime
