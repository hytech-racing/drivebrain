#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "RigidTransform2D.hpp"

namespace slam
{

enum class ConeColor
{
    Unknown,
    Blue,
    Yellow,
    OrangeSmall,
    OrangeBig,
};

// Step 1: Perception (LidarProcessor and/or cameras) -> frontend
struct ConeDetection
{
    transforms::Point2D position_base_m{};

    // Overall/fused confidence that this detection should be considered by the
    // frontend. Lidar-only detections can provide this directly (will be
    // Unknown); future vision can update it before the frame reaches
    // SlamFrontend
    double confidence{};

    ConeColor color{ConeColor::Unknown};
    double color_confidence{};
};

// Producer: Perception (LidarProcessor and/or cameras)
// Consumer: SlamFrontend::process_frame()
struct ConeFrame
{
    std::int64_t timestamp_ns{};
    transforms::Pose2D pose_odom_from_base{};
    std::vector<ConeDetection> detections{};
};

// Step 2: backend -> frontend feedback
struct MapLandmark
{
    std::uint64_t landmark_id{};
    transforms::Point2D position_map_m{};
};

// Producer: SLAM backend snapshot/update output
// Consumer: SlamFrontend::update_map_state()
struct MapState
{
    std::uint64_t sequence{};
    std::int64_t timestamp_ns{};
    transforms::Pose2D pose_map_from_odom{};

    // Complete current optimized map, not an incremental delta
    std::vector<MapLandmark> landmarks{};
};

enum class LandmarkAssociation
{
    ExistingMapLandmark,
    PendingLandmark,
    NewLandmark,
};

// Step 3: frontend -> runtime/backend handoff
struct LandmarkObservation
{
    std::uint64_t landmark_id{};
    transforms::Point2D measurement_base_m{};

    // Consumed by the backend to decide whether this observation may create a
    // new graph landmark or must reference an existing graph landmark
    LandmarkAssociation association{};

    // Frontend diagnostic metadata; not used as a graph measurement
    double residual_m{};
};

struct FrontendDiagnostics
{
    std::size_t detections_received{};
    std::size_t detections_rejected_invalid{};

    std::size_t optimized_associations{};
    std::size_t pending_associations{};
    std::size_t tentative_associations{};

    std::size_t tentative_tracks_created{};
    std::size_t tracks_promoted{};
    std::size_t tracks_removed_stale{};

    std::size_t unmatched_detection_count{};
};

// Producer: SlamFrontend::process_frame()
// Consumer: runtime handoff to LandmarkFrame, debug/telemetry publishers
struct FrontendResult
{
    bool frame_accepted{};
    std::string message{};

    std::int64_t timestamp_ns{};
    transforms::Pose2D pose_odom_from_base{};

    std::vector<LandmarkObservation> landmark_observations{};

    // Diagnostic metadata. The authoritative backend input is
    // landmark_observations
    std::vector<std::uint64_t> new_landmark_ids{};

    FrontendDiagnostics debug{};
};

// Step 4: frontend -> backend
// Producer: FrontendResult plus frame sequencing
// Consumer: IncrementalGraphSlam::process_frame()
struct LandmarkFrame
{
    std::uint64_t frame_index{};
    std::int64_t timestamp_ns{};

    // Raw measured T_odom_base used as odometry input to graph
    transforms::Pose2D recorded_pose_odom_from_base{};

    std::vector<LandmarkObservation> observations{};
};

// Step 5: backend configuration
struct PriorPoseNoise
{
    double x_std_m{};
    double y_std_m{};
    double yaw_std_rad{};
};

struct OdometryPoseNoise
{
    double x_std_m{};
    double y_std_m{};
    double yaw_std_rad{};
};

struct LandmarkMeasurementNoise
{
    double bearing_std_rad{};
    double range_std_m{};
};

struct MeasurementRange
{
    double max_m{};
    double min_m{};
};

// Step 6: backend -> runtime/debug output
// Producer: SLAM backend update/snapshot
// Consumer: runtime map-state feedback, debug/telemetry publishers
struct PoseEstimate
{
    std::size_t pose_index{};
    std::uint64_t frame_index{};
    std::int64_t timestamp_ns{};

    transforms::Pose2D initial_pose_map_from_base{};
    transforms::Pose2D recorded_pose_odom_from_base{};
    transforms::Pose2D optimized_pose_map_from_base{};
};

struct LandmarkEstimate
{
    std::uint64_t landmark_id{};
    transforms::Point2D initial_position_map{};
    transforms::Point2D optimized_position_map{};
};

}  // namespace slam
