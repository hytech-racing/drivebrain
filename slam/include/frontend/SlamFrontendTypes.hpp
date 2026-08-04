#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "common/SlamInterfaces.hpp"
#include "frontend/LandmarkColor.hpp"

namespace slam::frontend
{

enum class LocalTrackState
{
    Tentative,
    Pending,
};

// frontend-owned tentative or pending state
struct LocalLandmarkTrack
{
    std::uint64_t local_track_id{};
    LocalTrackState state{LocalTrackState::Tentative};

    // Empty while tentative; assigned at promotion
    std::optional<std::uint64_t> landmark_id{};
    transforms::Point2D position_odom_m{};

    std::size_t observation_count{};
    std::int64_t first_seen_ns{};
    std::int64_t last_seen_ns{};

    ColorEvidence color_evidence{};
};

struct ValidDetection
{
    // index into frame.observations
    std::size_t source_observation_index{};

    transforms::Point2D measurement_base_m{};
    transforms::Point2D position_odom_m{};

    double confidence{};
    ConeColor color{ConeColor::Unknown};
    double color_confidence{};
};

struct LocalTrackTarget
{
    std::size_t local_track_index{};
    transforms::Point2D position_odom_m{};
};

struct AssociationCandidate
{
    // index into valid_detection
    std::size_t valid_detection_index{};

    // index into the target view supplied to candidate construction:
    // predicted_landmarks, pending_targets, or tentative_targets
    std::size_t target_view_index{};

    double residual_m{};
};

struct AcceptedAssociation
{
    // index into valid_detection
    std::size_t valid_detection_index{};

    // index into the target view supplied to candidate construction:
    // predicted_landmarks, pending_targets, or tentative_targets
    std::size_t target_view_index{};

    double residual_m{};
};

struct PredictedLandmarkMeasurement
{
    std::uint64_t landmark_id{};
    transforms::Point2D predicted_measurement_base_m{};
};

struct MapStateUpdateResult
{
    bool accepted{};
    std::string message{};

    std::size_t landmark_count{};
    std::size_t pending_tracks_resolved{};
};

struct SlamFrontendParams
{
    double optimized_association_gate_m{};
    double local_track_association_gate_m{};

    std::size_t minimum_observations_to_confirm{};

    std::int64_t tentative_track_max_age_ns{};
    std::int64_t pending_track_max_age_ns{};

    double minimum_detection_confidence{};
};

struct PendingPlannerLandmark
{
    std::uint64_t landmark_id{};
    transforms::Point2D position_odom_m{};
    LandmarkColorEstimate color_estimate{};
};

}  // namespace slam::frontend
