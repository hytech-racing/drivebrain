#include "PerceptionFrontendRunner.hpp"

#include <spdlog/spdlog.h>

#include <limits>
#include <stdexcept>

#include "PerceptionDebugMessageAdapters.hpp"
#include "PointCloudMessageAdapters.hpp"
#include "SlamDebugMessageAdapters.hpp"
#include "SlamVisualizationAdapters.hpp"
#include "Telemetry.hpp"
#include "common/SlamInterfaces.hpp"
#include "dv_msgs.pb.h"

namespace runtime
{
namespace
{
using namespace std::chrono_literals;

constexpr const char* kLidarFrame = "lidar";
constexpr const auto kTransformBufferTimeout = 5ms;

std::uint32_t to_debug_count(const std::size_t count)
{
    return count > std::numeric_limits<std::uint32_t>::max()
               ? std::numeric_limits<std::uint32_t>::max()
               : static_cast<std::uint32_t>(count);
}

slam::ConeFrame cone_candidates_to_cone_frame(
    const std::vector<perception::ConeCandidate>& cone_candidates,
    const transforms::Pose3D pose_odom_from_base,
    const transforms::Pose3D pose_base_from_lidar,
    const std::int64_t timestamp_ns)
{
    slam::ConeFrame cone_frame;

    cone_frame.timestamp_ns = timestamp_ns;
    cone_frame.pose_odom_from_base = pose_odom_from_base.to_pose2d();

    if (cone_candidates.empty())
    {
        return cone_frame;
    }

    cone_frame.detections.reserve(cone_candidates.size());

    for (const perception::ConeCandidate& candidate : cone_candidates)
    {
        slam::ConeDetection detection;
        detection.confidence = candidate.confidence;
        detection.color = slam::ConeColor::Unknown;
        detection.color_confidence = 0.0;

        const transforms::Point3D point_lidar{
            candidate.position.x, candidate.position.y, candidate.position.z};
        const transforms::Point3D point_base =
            pose_base_from_lidar * point_lidar;

        detection.position_base_m.x_m = point_base.x_m;
        detection.position_base_m.y_m = point_base.y_m;

        cone_frame.detections.push_back(detection);
    }

    return cone_frame;
}

slam::LandmarkFrame frontend_to_landmark_frame(
    const slam::FrontendResult& frontend_result, std::uint64_t frame_index)
{
    return slam::LandmarkFrame{frame_index, frontend_result.timestamp_ns,
                               frontend_result.pose_odom_from_base,
                               frontend_result.landmark_observations};
}

}  // namespace

PerceptionFrontendRunner::PerceptionFrontendRunner(
    std::shared_ptr<transforms::TransformBuffer> transform_buffer,
    std::shared_ptr<slam::LatestMapState> latest_map_state,
    LandmarkFrameHandler landmark_frame_handler,
    perception::LidarProcessorParams lidar_processor_params,
    slam::frontend::SlamFrontendParams slam_frontend_params,
    bool publish_full_telemetry,
    std::shared_ptr<slam::LatestPlannerMap> latest_planner_map)
    : _transform_buffer(std::move(transform_buffer)),
      _latest_map_state(std::move(latest_map_state)),
      _latest_planner_map(std::move(latest_planner_map)),
      _landmark_frame_handler(std::move(landmark_frame_handler)),
      _lidar_processor_params(lidar_processor_params),
      _lidar_processor(_lidar_processor_params),
      _frontend_params(slam_frontend_params),
      _slam_frontend(_frontend_params),
      _publish_full_telemetry(publish_full_telemetry)
{
    if (!_transform_buffer)
    {
        throw std::invalid_argument(
            "PerceptionFrontendRunner requires TransformBuffer");
    }

    if (!_latest_map_state)
    {
        throw std::invalid_argument(
            "PerceptionFrontendRunner requires LatestMapState");
    }

    if (!_landmark_frame_handler)
    {
        throw std::invalid_argument(
            "PerceptionFrontendRunner requires LandmarkFrameHandler");
    }
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

        if (_point_cloud_queue_stats.point_clouds_enqueued > 0U &&
            stamped_point_cloud.timestamp_ns <=
                _point_cloud_queue_stats.latest_point_cloud_timestamp_ns)
        {
            _point_cloud_queue_stats.nonincreasing_point_clouds++;
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
            else
            {
                _point_cloud_queue_stats.point_clouds_processing_failed++;
            }
        }

        if (_publish_full_telemetry)
        {
            _publish_runner_debug(stamped_point_cloud.timestamp_ns);
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

    if (!lidar_processor_result)
    {
        return false;
    }

    const auto pose_odom_from_base = _transform_buffer->lookup3d(
        transforms::FrameId::Odom, transforms::FrameId::Baselink,
        lidar_processor_result->timestamp_ns, kTransformBufferTimeout);

    if (!pose_odom_from_base)
    {
        return false;
    }

    const transforms::Pose3D pose_base_from_lidar =
        _transform_buffer->T_base_lidar3d();

    const slam::ConeFrame cone_frame = cone_candidates_to_cone_frame(
        lidar_processor_result->cone_candidates, *pose_odom_from_base,
        pose_base_from_lidar, lidar_processor_result->timestamp_ns);

    _apply_latest_map_state();

    const slam::FrontendResult frontend_result =
        _slam_frontend.process_frame(cone_frame);

    if (!frontend_result.frame_accepted)
    {
        return false;
    }

    const std::optional<slam::PlannerMap> planner_map =
        _slam_frontend.planner_map(_next_planner_map_sequence,
                                   frontend_result.timestamp_ns);
    if (planner_map)
    {
        if (_latest_planner_map)
        {
            _latest_planner_map->store(*planner_map);
        }
        _next_planner_map_sequence++;
    }

    slam::LandmarkFrame landmark_frame =
        frontend_to_landmark_frame(frontend_result, _next_landmark_frame_index);

    if (!_landmark_frame_handler(std::move(landmark_frame)))
    {
        spdlog::error("Failed to enqueue SLAM LandmarkFrame at timestamp {}",
                      frontend_result.timestamp_ns);
        return false;
    }

    _next_landmark_frame_index++;

    if (_publish_full_telemetry)
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

        core::log("/slam/frontend/associated_observations",
                  adapters::to_foxglove_frontend_association_markers(
                      frontend_result, "base_link"));

        core::log("/slam/frontend/association_text",
                  adapters::to_foxglove_frontend_association_text(
                      frontend_result, "base_link"));

        if (planner_map)
        {
            core::log("/mapping/planner_landmarks",
                      adapters::to_foxglove_planner_landmark_markers(
                          *planner_map));
            core::log("/mapping/planner_landmark_text",
                      adapters::to_foxglove_planner_landmark_text(*planner_map));
        }

        _publish_slam_frontend_debug(frontend_result);
    }

    return true;
}

void PerceptionFrontendRunner::_apply_latest_map_state()
{
    const std::optional<slam::MapState> map_state = _latest_map_state->latest();

    if (!map_state)
    {
        return;
    }

    if (_last_consumed_map_sequence &&
        map_state->sequence <= *_last_consumed_map_sequence)
    {
        return;
    }

    const slam::frontend::MapStateUpdateResult update_result =
        _slam_frontend.update_map_state(*map_state);

    // consume regardless of acceptance status
    _last_consumed_map_sequence = map_state->sequence;

    if (!update_result.accepted)
    {
        _map_state_update_stats.update_failed++;
    }
    else
    {
        _map_state_update_stats.update_success++;
    }

    _map_state_update_stats.message = update_result.message;
    _map_state_update_stats.landmark_count = update_result.landmark_count;
    _map_state_update_stats.pending_tracks_resolved =
        update_result.pending_tracks_resolved;
}

void PerceptionFrontendRunner::_publish_slam_frontend_debug(
    const slam::FrontendResult& frontend_result) const
{
    core::log("/slam/frontend/debug",
              adapters::to_slam_frontend_debug(frontend_result));
}

void PerceptionFrontendRunner::_publish_runner_debug(
    const std::int64_t timestamp_ns) const
{
    auto message = std::make_shared<dv_msgs::SlamFrontendRunnerDebug>();
    message->set_timestamp_ns(timestamp_ns);

    const PointCloudQueueStats queue_stats = point_cloud_queue_stats();
    message->set_point_clouds_enqueued(
        to_debug_count(queue_stats.point_clouds_enqueued));
    message->set_point_clouds_processed(
        to_debug_count(queue_stats.point_clouds_processed));
    message->set_point_clouds_processing_failed(
        to_debug_count(queue_stats.point_clouds_processing_failed));
    message->set_nonincreasing_point_clouds(
        to_debug_count(queue_stats.nonincreasing_point_clouds));
    message->set_queue_trims(to_debug_count(queue_stats.queue_trims));
    message->set_current_queue_depth(
        to_debug_count(queue_stats.current_queue_depth));
    message->set_maximum_queue_depth(
        to_debug_count(queue_stats.maximum_queue_depth));
    message->set_latest_point_cloud_timestamp_ns(
        queue_stats.latest_point_cloud_timestamp_ns);

    message->set_map_state_update_success(
        to_debug_count(_map_state_update_stats.update_success));
    message->set_map_state_update_failed(
        to_debug_count(_map_state_update_stats.update_failed));
    message->set_map_state_message(_map_state_update_stats.message);
    message->set_map_state_landmark_count(
        to_debug_count(_map_state_update_stats.landmark_count));
    message->set_map_state_pending_tracks_resolved(
        to_debug_count(_map_state_update_stats.pending_tracks_resolved));

    core::log("/slam/frontend/runner_debug", message);
}

}  // namespace runtime
