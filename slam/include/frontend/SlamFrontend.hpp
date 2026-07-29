#pragma once
#include <cstdint>
#include <optional>
#include <unordered_set>
#include <vector>

#include "SlamFrontendTypes.hpp"

namespace slam::frontend
{

class SlamFrontend
{
   public:
    explicit SlamFrontend(const SlamFrontendParams& params);

    [[nodiscard]] MapStateUpdateResult update_map_state(
        const slam::MapState& state);

    [[nodiscard]] slam::FrontendResult process_frame(
        const slam::ConeFrame& frame);

   public:
    // Test file only, do not call unless for testing
    [[nodiscard]]
    std::vector<AcceptedAssociation> associate_points_one_to_one(
        const std::vector<transforms::Point2D>& detections,
        const std::vector<transforms::Point2D>& targets,
        const double gate_m) const;

   private:
    // update_map_state helpers
    bool _map_state_is_ok(const slam::MapState& state,
                          std::string& rejection_message) const;

    std::unordered_set<std::uint64_t> _find_acknowledged_pending_landmark_ids(
        const slam::MapState& state) const;

    void _remove_acknowledged_pending_tracks(
        const std::unordered_set<std::uint64_t>& landmark_ids);

    bool _advance_landmark_id_allocator(const slam::MapState& state,
                                         std::string& rejection_message);

   private:
    // Pure geometry
    [[nodiscard]] std::vector<PredictedLandmarkMeasurement>
    _predict_optimized_landmarks_in_base(
        const slam::MapState& map_state,
        const transforms::Pose2D& pose_odom_from_base) const;

    // Pure association decisions
    [[nodiscard]]
    std::vector<AssociationCandidate> _build_optimized_association_candidates(
        const std::vector<ValidDetection>& valid_detections,
        const std::vector<PredictedLandmarkMeasurement>& predicted_landmarks,
        double gate_m) const;

    [[nodiscard]]
    std::vector<AssociationCandidate> _build_association_candidates(
        const std::vector<ValidDetection>& valid_detections,
        const std::vector<LocalTrackTarget>& targets, double gate_m) const;

    [[nodiscard]]
    std::vector<AcceptedAssociation> _select_one_to_one_associations(
        std::vector<AssociationCandidate> candidates) const;

   private:
    // Optimized association helper
    void _emit_optimized_associations(
        const std::vector<ValidDetection>& valid_detections,
        const std::vector<PredictedLandmarkMeasurement>& predicted_landmarks,
        const std::vector<AcceptedAssociation>& optimized_associations,
        slam::FrontendResult& result);

   private:
    // Persistent-state mutation
    void _apply_tentative_track_associations(
        const slam::ConeFrame& frame,
        const std::vector<ValidDetection>& valid_detections,
        const std::vector<LocalTrackTarget>& tentative_targets,
        const std::vector<AcceptedAssociation>& associations);

    void _apply_pending_track_associations(
        const slam::ConeFrame& frame,
        const std::vector<ValidDetection>& valid_detections,
        const std::vector<LocalTrackTarget>& pending_targets,
        const std::vector<AcceptedAssociation>& associations,
        slam::FrontendResult& result);

    void _create_tentative_tracks(
        const slam::ConeFrame& frame,
        const std::vector<ValidDetection>& valid_detections,
        const std::vector<std::size_t>& unmatched_valid_detection_indices);

    void _promote_eligible_tracks(
        const slam::ConeFrame& frame,
        const std::vector<ValidDetection>& valid_detections,
        const std::vector<LocalTrackTarget>& tentative_targets,
        const std::vector<AcceptedAssociation>& tentative_associations,
        slam::FrontendResult& result);

    void _remove_stale_local_tracks(std::int64_t current_timestamp_ns,
                                    slam::FrontendDiagnostics& diagnostics);

   private:
    SlamFrontendParams _params;

    std::optional<slam::MapState> _latest_map_state;
    std::vector<LocalLandmarkTrack> _local_tracks;

    std::uint64_t _next_local_track_id{};
    std::uint64_t _next_landmark_id{};

    std::optional<std::uint64_t> _cached_map_sequence;
    std::optional<std::int64_t> _previous_map_state_timestamp_ns;
    std::optional<std::int64_t> _previous_frame_timestamp_ns;
};

}  // namespace slam::frontend
