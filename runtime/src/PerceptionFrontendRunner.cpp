#include "PerceptionFrontendRunner.hpp"

#include <spdlog/spdlog.h>

#include "PerceptionDebugMessageAdapters.hpp"
#include "PointCloudMessageAdapters.hpp"
#include "Telemetry.hpp"

namespace runtime
{
namespace
{

constexpr const char* kLidarFrame = "lidar";

}  // namespace

PerceptionFrontendRunner::PerceptionFrontendRunner(
    std::shared_ptr<transforms::TransformBuffer> transform_buffer,
    perception::LidarProcessorParams lidar_processor_params,
    bool publish_full_telemetry)
    : _transform_buffer(transform_buffer),
      _lidar_processor_params(lidar_processor_params),
      _lidar_processor(_lidar_processor_params),
      _publish_full_telemetry(publish_full_telemetry)
{
}

PerceptionFrontendRunner::~PerceptionFrontendRunner() { stop(); }

void PerceptionFrontendRunner::start()
{
    {
        std::scoped_lock lock(_point_cloud_queue_mutex);

        if (_point_cloud_processor_running)
        {
            return;
        }

        _point_cloud_processor_running = true;
    }

    _point_cloud_processor_thread =
        std::thread(&PerceptionFrontendRunner::_run, this);
}

void PerceptionFrontendRunner::stop()
{
    if (!_point_cloud_processor_running)
    {
        return;
    }

    {
        std::scoped_lock lock(_point_cloud_queue_mutex);
        _point_cloud_processor_running = false;
    }

    _point_cloud_queue_cv.notify_one();

    if (_point_cloud_processor_thread.joinable())
    {
        _point_cloud_processor_thread.join();
    }
}

// Hello me, do NOT try to access the point cloud after this call
bool PerceptionFrontendRunner::enqueue(
    perception::StampedPointCloud&& stamped_point_cloud)
{
    {
        std::scoped_lock lock(_point_cloud_queue_mutex,
                              _point_cloud_queue_stats_mutex);

        if (stamped_point_cloud.timestamp_ns <
            _point_cloud_queue_stats.latest_point_cloud_timestamp_ns)
        {
            _point_cloud_queue_stats.out_of_order_point_clouds++;
            return false;
        }

        if (_point_cloud_queue.size() >= kMaximumQueueSize)
        {
            // trim the oldest point cloud
            _point_cloud_queue.pop_front();
            _point_cloud_queue_stats.queue_trims++;
        }

        _point_cloud_queue_stats.latest_point_cloud_timestamp_ns =
            stamped_point_cloud.timestamp_ns;
        _point_cloud_queue_stats.point_clouds_enqueued++;

        _point_cloud_queue.push_back(std::move(stamped_point_cloud));

        _point_cloud_queue_stats.current_queue_depth =
            _point_cloud_queue.size();
        _point_cloud_queue_stats.maximum_queue_depth =
            std::max(_point_cloud_queue_stats.maximum_queue_depth,
                     _point_cloud_queue_stats.current_queue_depth);
    }

    _point_cloud_queue_cv.notify_one();
    return true;
}

PointCloudQueueStats PerceptionFrontendRunner::point_cloud_queue_stats() const
{
    std::scoped_lock lock(_point_cloud_queue_mutex,
                          _point_cloud_queue_stats_mutex);
    PointCloudQueueStats stats = _point_cloud_queue_stats;
    stats.current_queue_depth = _point_cloud_queue.size();
    return stats;
}

void PerceptionFrontendRunner::_run()
{
    while (true)
    {
        perception::StampedPointCloud stamped_point_cloud;

        {
            std::unique_lock point_cloud_queue_lock(_point_cloud_queue_mutex);

            _point_cloud_queue_cv.wait(
                point_cloud_queue_lock,
                [this]()
                {
                    return !_point_cloud_processor_running ||
                           !_point_cloud_queue.empty();
                });

            if (!_point_cloud_processor_running && _point_cloud_queue.empty())
            {
                return;
            }

            stamped_point_cloud = std::move(_point_cloud_queue.front());
            _point_cloud_queue.pop_front();

            {
                std::scoped_lock point_cloud_stats_lock(
                    _point_cloud_queue_stats_mutex);
                _point_cloud_queue_stats.current_queue_depth =
                    _point_cloud_queue.size();
            }
        }

        const bool processed = _process_point_cloud(stamped_point_cloud);

        {
            std::scoped_lock point_cloud_stats_lock(
                _point_cloud_queue_stats_mutex);

            if (processed)
            {
                _point_cloud_queue_stats.point_clouds_processed++;
            }
        }
    }
}

bool PerceptionFrontendRunner::_process_point_cloud(
    const perception::StampedPointCloud& stamped_point_cloud)
{
    if (stamped_point_cloud.frame != transforms::FrameId::Lidar)
    {
        spdlog::warn("Rejected point cloud in non-lidar frame");
        return false;
    }

    perception::LidarProcessingInput lidar_processor_input{stamped_point_cloud};

    const auto lidar_processor_result =
        _lidar_processor.process(lidar_processor_input);

    if (_publish_full_telemetry && lidar_processor_result)
    {
        core::log("/perception/lidar_processing_debug",
                  adapters::to_lidar_processing_debug(stamped_point_cloud,
                                                      *lidar_processor_result));

        core::log(
            "/perception/deskewed_point_cloud",
            adapters::to_foxglove_point_cloud(
                lidar_processor_result->deskewed_point_cloud, kLidarFrame));

        core::log(
            "/perception/filtered_point_cloud",
            adapters::to_foxglove_point_cloud(
                lidar_processor_result->filtered_point_cloud, kLidarFrame));

        core::log("/perception/ground_point_cloud",
                  adapters::to_foxglove_point_cloud(
                      lidar_processor_result->ground_point_cloud, kLidarFrame));

        core::log(
            "/perception/non_ground_point_cloud",
            adapters::to_foxglove_point_cloud(
                lidar_processor_result->non_ground_point_cloud, kLidarFrame));

        core::log(
            "/perception/cluster_markers",
            adapters::to_foxglove_cluster_markers(
                lidar_processor_result->cluster_features, kLidarFrame,
                lidar_processor_result->non_ground_point_cloud.timestamp_ns));

        core::log(
            "/perception/cone_candidate_markers",
            adapters::to_foxglove_cone_candidate_markers(
                lidar_processor_result->cone_candidates, kLidarFrame,
                lidar_processor_result->non_ground_point_cloud.timestamp_ns));

        core::log(
            "/perception/cone_candidate_text",
            adapters::to_foxglove_cone_candidate_text(
                lidar_processor_result->cone_candidates, kLidarFrame,
                lidar_processor_result->non_ground_point_cloud.timestamp_ns));

        core::log(
            "/perception/rejected_cluster_markers",
            adapters::to_foxglove_rejected_cluster_markers(
                lidar_processor_result->rejected_clusters, kLidarFrame,
                lidar_processor_result->non_ground_point_cloud.timestamp_ns));

        core::log(
            "/perception/rejected_cluster_text",
            adapters::to_foxglove_rejected_cluster_text(
                lidar_processor_result->rejected_clusters, kLidarFrame,
                lidar_processor_result->non_ground_point_cloud.timestamp_ns));
    }

    return true;
}

}  // namespace runtime
