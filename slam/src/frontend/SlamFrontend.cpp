
#include "frontend/SlamFrontend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <tuple>
#include <unordered_set>

namespace slam::frontend
{

namespace
{

bool _params_are_ok(const SlamFrontendParams& params)
{
    const auto positive_and_finite = [](const double value)
    { return std::isfinite(value) && value > 0.0; };

    if (!positive_and_finite(params.optimized_association_gate_m) ||
        !positive_and_finite(params.local_track_association_gate_m))
    {
        return false;
    }

    if (params.minimum_observations_to_confirm < 2U)
    {
        return false;
    }

    if (params.tentative_track_max_age_ns <= 0 ||
        params.pending_track_max_age_ns <= 0)
    {
        return false;
    }

    if (!std::isfinite(params.minimum_detection_confidence) ||
        params.minimum_detection_confidence < 0.0 ||
        params.minimum_detection_confidence > 1.0)
    {
        return false;
    }

    return true;
}

}  // namespace

SlamFrontend::SlamFrontend(const SlamFrontendParams& params)
{
    if (!_params_are_ok(params))
    {
        throw std::invalid_argument("SLAM front end params are invalid");
    }

    _params = params;
    _next_local_track_id = 0;
    _next_landmark_id = 0;
}

MapStateUpdateResult SlamFrontend::update_map_state(
    const MapState& state)
{
    MapStateUpdateResult result;

    if (!_map_state_is_ok(state, result.message))
    {
        result.accepted = false;
        return result;
    }

    if (!_advance_landmark_id_allocator(state, result.message))
    {
        result.accepted = false;
        return result;
    }

    std::unordered_set<std::uint64_t> acknowledged_pending_track_landmark_ids =
        _find_acknowledged_pending_landmark_ids(state);

    const std::size_t local_track_size_before = _local_tracks.size();

    _remove_acknowledged_pending_tracks(
        acknowledged_pending_track_landmark_ids);

    const std::size_t local_track_size_after = _local_tracks.size();

    _latest_map_state = state;
    _previous_map_state_timestamp_ns = state.timestamp_ns;
    _cached_map_sequence = state.sequence;

    result.accepted = true;
    result.landmark_count = state.landmarks.size();
    result.pending_tracks_resolved =
        local_track_size_before - local_track_size_after;
    result.message = "Map state accepted";

    return result;
}

bool SlamFrontend::_map_state_is_ok(const MapState& state,
                                    std::string& rejection_message) const
{
    if (!std::isfinite(state.pose_map_from_odom.x_m) ||
        !std::isfinite(state.pose_map_from_odom.y_m) ||
        !std::isfinite(state.pose_map_from_odom.yaw_rad))
    {
        rejection_message = "Map state pose is not finite";
        return false;
    }

    if (state.timestamp_ns < 0)
    {
        rejection_message = "Map state timestamp must be nonnegative";
        return false;
    }

    if (_previous_map_state_timestamp_ns &&
        state.timestamp_ns <= *_previous_map_state_timestamp_ns)
    {
        rejection_message = "Map state timestamp is not strictly increasing";
        return false;
    }

    if (_cached_map_sequence && state.sequence <= *_cached_map_sequence)
    {
        rejection_message = "Map estimate sequence is not strictly increasing";
        return false;
    }

    std::unordered_set<std::uint64_t> seen;

    for (const MapLandmark& landmark : state.landmarks)
    {
        if (!std::isfinite(landmark.position_map_m.x_m) ||
            !std::isfinite(landmark.position_map_m.y_m))
        {
            rejection_message = "Landmark position is not finite";
            return false;
        }

        if (!seen.insert(landmark.landmark_id).second)
        {
            rejection_message = "Map state landmark ids are not unique";
            return false;
        }
    }

    return true;
}

std::unordered_set<std::uint64_t>
SlamFrontend::_find_acknowledged_pending_landmark_ids(
    const MapState& state) const
{
    std::unordered_set<std::uint64_t> acknowledged_pending_landmark_ids;

    if (state.landmarks.empty() || _local_tracks.empty())
    {
        return acknowledged_pending_landmark_ids;
    }

    acknowledged_pending_landmark_ids.reserve(_local_tracks.size());

    std::unordered_set<std::uint64_t> pending_track_landmark_ids;

    for (const LocalLandmarkTrack& track : _local_tracks)
    {
        if (track.state == LocalTrackState::Pending)
        {
            if (!track.landmark_id.has_value())
            {
                throw std::logic_error("Pending track has no landmark ID");
            }

            pending_track_landmark_ids.insert(*track.landmark_id);
        }
    }

    for (const MapLandmark& landmark : state.landmarks)
    {
        if (pending_track_landmark_ids.count(landmark.landmark_id) == 1)
        {
            acknowledged_pending_landmark_ids.insert(landmark.landmark_id);
        }
    }

    return acknowledged_pending_landmark_ids;
}

void SlamFrontend::_remove_acknowledged_pending_tracks(
    const std::unordered_set<std::uint64_t>& landmark_ids)
{
    _local_tracks.erase(
        std::remove_if(_local_tracks.begin(), _local_tracks.end(),
                       [&landmark_ids](const LocalLandmarkTrack& track)
                       {
                           if (track.state != LocalTrackState::Pending)
                           {
                               return false;
                           }

                           if (!track.landmark_id.has_value())
                           {
                               throw std::logic_error(
                                   "Pending track has no landmark ID");
                           }

                           return landmark_ids.count(*track.landmark_id) > 0U;
                       }),
        _local_tracks.end());
}

bool SlamFrontend::_advance_landmark_id_allocator(
    const MapState& state, std::string& rejection_message)
{
    std::uint64_t candidate_next_landmark_id = _next_landmark_id;
    for (const MapLandmark& landmark : state.landmarks)
    {
        if (landmark.landmark_id == std::numeric_limits<std::uint64_t>::max())
        {
            rejection_message = "Map state landmark ID exhausts the allocator";
            return false;
        }

        candidate_next_landmark_id =
            std::max(candidate_next_landmark_id, landmark.landmark_id + 1U);
    }

    _next_landmark_id = candidate_next_landmark_id;

    return true;
}

FrontendResult SlamFrontend::process_frame(const ConeFrame& frame)
{
    FrontendResult result;

    if (frame.timestamp_ns < 0)
    {
        result.frame_accepted = false;
        result.message = "ConeFrame timestamp must be nonnegative";
        return result;
    }

    if (_previous_frame_timestamp_ns &&
        frame.timestamp_ns <= *_previous_frame_timestamp_ns)
    {
        result.frame_accepted = false;
        result.message = "ConeFrame timestamp not strictly increasing";
        return result;
    }

    result.timestamp_ns = frame.timestamp_ns;

    if (!std::isfinite(frame.pose_odom_from_base.x_m) ||
        !std::isfinite(frame.pose_odom_from_base.y_m) ||
        !std::isfinite(frame.pose_odom_from_base.yaw_rad))
    {
        result.frame_accepted = false;
        result.message = "ConeFrame pose_odom_from_base not finite";
        return result;
    }

    result.pose_odom_from_base = frame.pose_odom_from_base;

    std::vector<ValidDetection> valid_detections;
    valid_detections.reserve(frame.detections.size());
    for (std::size_t observation_index = 0;
         observation_index < frame.detections.size(); ++observation_index)
    {
        const ConeDetection& observation = frame.detections[observation_index];

        if (!std::isfinite(observation.confidence) ||
            !std::isfinite(observation.position_base_m.x_m) ||
            !std::isfinite(observation.position_base_m.y_m) ||
            observation.confidence < 0.0 || observation.confidence > 1.0)
        {
            continue;
        }

        if (observation.confidence >= _params.minimum_detection_confidence)
        {
            valid_detections.push_back(ValidDetection{
                observation_index,
                observation.position_base_m,
                frame.pose_odom_from_base.transform_point(
                    observation.position_base_m),
                observation.confidence,
            });
        }
    }

    std::vector<std::size_t> unmatched_valid_detection_indices;
    std::vector<bool> valid_detection_used(valid_detections.size(), false);

    std::vector<AcceptedAssociation> optimized_associations;

    const bool map_state_is_usable =
            _latest_map_state.has_value() &&
        _latest_map_state->timestamp_ns <= frame.timestamp_ns;

    // Associate with optimized landmarks if map state is usable
    if (map_state_is_usable)
    {
        const std::vector<PredictedLandmarkMeasurement> predicted_landmarks =
            _predict_optimized_landmarks_in_base(*_latest_map_state,
                                                frame.pose_odom_from_base);
        const std::vector<AssociationCandidate> optimized_candidates =
            _build_optimized_association_candidates(
                valid_detections, predicted_landmarks,
                _params.optimized_association_gate_m);
        optimized_associations =
            _select_one_to_one_associations(optimized_candidates);
        _emit_optimized_associations(valid_detections, predicted_landmarks,
                                    optimized_associations, result);

        for (const AcceptedAssociation& association : optimized_associations)
        {
            valid_detection_used.at(association.valid_detection_index) = true;
        }
    }

    // Associate with pending tracks
    std::vector<LocalTrackTarget> pending_targets;
    pending_targets.reserve(_local_tracks.size());

    for (std::size_t local_track_index = 0;
         local_track_index < _local_tracks.size(); ++local_track_index)
    {
        const LocalLandmarkTrack& track = _local_tracks[local_track_index];
        if (track.state == LocalTrackState::Pending)
        {
            pending_targets.push_back(
                LocalTrackTarget{local_track_index, track.position_odom_m});
        }
    }

    std::vector<AssociationCandidate> pending_candidates =
        _build_association_candidates(valid_detections, pending_targets,
                                     _params.local_track_association_gate_m);

    pending_candidates.erase(
        std::remove_if(
            pending_candidates.begin(), pending_candidates.end(),
            [&valid_detection_used](const AssociationCandidate& candidate)
            {
                return valid_detection_used.at(candidate.valid_detection_index);
            }),
        pending_candidates.end());

    const std::vector<AcceptedAssociation> pending_associations =
        _select_one_to_one_associations(pending_candidates);

    _apply_pending_track_associations(frame, valid_detections, pending_targets,
                                     pending_associations, result);

    for (const AcceptedAssociation& association : pending_associations)
    {
        valid_detection_used.at(association.valid_detection_index) = true;
    }

    // Associate with tentative tracks
    std::vector<LocalTrackTarget> tentative_targets;
    tentative_targets.reserve(_local_tracks.size());

    for (std::size_t local_track_index = 0;
         local_track_index < _local_tracks.size(); ++local_track_index)
    {
        const LocalLandmarkTrack& track = _local_tracks[local_track_index];
        if (track.state == LocalTrackState::Tentative)
        {
            tentative_targets.push_back(
                LocalTrackTarget{local_track_index, track.position_odom_m});
        }
    }

    std::vector<AssociationCandidate> tentative_candidates =
        _build_association_candidates(valid_detections, tentative_targets,
                                     _params.local_track_association_gate_m);

    tentative_candidates.erase(
        std::remove_if(
            tentative_candidates.begin(), tentative_candidates.end(),
            [&valid_detection_used](const AssociationCandidate& candidate)
            {
                return valid_detection_used.at(candidate.valid_detection_index);
            }),
        tentative_candidates.end());

    const std::vector<AcceptedAssociation> tentative_associations =
        _select_one_to_one_associations(tentative_candidates);

    _apply_tentative_track_associations(
        frame, valid_detections, tentative_targets, tentative_associations);

    for (const AcceptedAssociation& association : tentative_associations)
    {
        valid_detection_used.at(association.valid_detection_index) = true;
    }

    for (std::size_t valid_detection_index = 0;
         valid_detection_index < valid_detection_used.size();
         ++valid_detection_index)
    {
        if (!valid_detection_used[valid_detection_index])
        {
            unmatched_valid_detection_indices.push_back(valid_detection_index);
        }
    }

    _create_tentative_tracks(frame, valid_detections,
                            unmatched_valid_detection_indices);

    _promote_eligible_tracks(frame, valid_detections, tentative_targets,
                            tentative_associations, result);

    _remove_stale_local_tracks(frame.timestamp_ns, result.debug);

    result.debug.detections_received = frame.detections.size();

    result.debug.detections_rejected_invalid =
        result.debug.detections_received - valid_detections.size();

    result.debug.optimized_associations = optimized_associations.size();
    result.debug.pending_associations = pending_associations.size();
    result.debug.tentative_associations = tentative_associations.size();

    result.debug.tentative_tracks_created =
        unmatched_valid_detection_indices.size();

    result.debug.unmatched_detection_count =
        unmatched_valid_detection_indices.size();

    _previous_frame_timestamp_ns = frame.timestamp_ns;

    result.frame_accepted = true;
    result.message = "Success";

    return result;
}

std::vector<PredictedLandmarkMeasurement>
SlamFrontend::_predict_optimized_landmarks_in_base(
    const MapState& map_state,
    const transforms::Pose2D& pose_odom_from_base) const
{
    std::vector<PredictedLandmarkMeasurement> predicted_landmark_measurements;

    if (map_state.landmarks.empty())
    {
        return predicted_landmark_measurements;
    }

    const transforms::Pose2D pose_map_from_base =
        map_state.pose_map_from_odom.compose(pose_odom_from_base);
    const transforms::Pose2D pose_base_from_map = pose_map_from_base.inverse();

    for (const MapLandmark& landmark : map_state.landmarks)
    {
        PredictedLandmarkMeasurement predicted_landmark_measurement{
            landmark.landmark_id,
            pose_base_from_map.transform_point(landmark.position_map_m)};

        predicted_landmark_measurements.push_back(
            predicted_landmark_measurement);
    }

    return predicted_landmark_measurements;
}

std::vector<AssociationCandidate>
SlamFrontend::_build_optimized_association_candidates(
    const std::vector<ValidDetection>& valid_detections,
    const std::vector<PredictedLandmarkMeasurement>& predicted_landmarks,
    double gate_m) const
{
    std::vector<AssociationCandidate> candidates;

    if (valid_detections.empty() || predicted_landmarks.empty())
    {
        return candidates;
    }

    candidates.reserve(valid_detections.size() * predicted_landmarks.size());

    for (std::size_t valid_detection_index = 0;
         valid_detection_index < valid_detections.size();
         ++valid_detection_index)
    {
        const ValidDetection& valid_detection =
            valid_detections[valid_detection_index];
        const transforms::Point2D& measurement_base_m =
            valid_detection.measurement_base_m;

        for (std::size_t target_view_index = 0;
             target_view_index < predicted_landmarks.size();
             ++target_view_index)
        {
            const transforms::Point2D& prediction_base_m =
                predicted_landmarks[target_view_index]
                    .predicted_measurement_base_m;

            const double distance_m =
                std::hypot(prediction_base_m.x_m - measurement_base_m.x_m,
                           prediction_base_m.y_m - measurement_base_m.y_m);

            if (distance_m > gate_m)
            {
                continue;
            }

            candidates.push_back(AssociationCandidate{
                valid_detection_index, target_view_index, distance_m});
        }
    }

    return candidates;
}

std::vector<AssociationCandidate> SlamFrontend::_build_association_candidates(
    const std::vector<ValidDetection>& valid_detections,
    const std::vector<LocalTrackTarget>& targets, double gate_m) const
{
    std::vector<AssociationCandidate> association_candidates;

    if (valid_detections.empty() || targets.empty())
    {
        return association_candidates;
    }

    association_candidates.reserve(valid_detections.size() * targets.size());

    for (std::size_t i = 0; i < valid_detections.size(); ++i)
    {
        const transforms::Point2D& detection_point =
            valid_detections[i].position_odom_m;

        for (std::size_t j = 0; j < targets.size(); ++j)
        {
            const transforms::Point2D& target_point =
                targets[j].position_odom_m;
            const double distance_m =
                std::hypot(target_point.x_m - detection_point.x_m,
                           target_point.y_m - detection_point.y_m);
            if (distance_m <= gate_m)
            {
                association_candidates.push_back(
                    AssociationCandidate{i, j, distance_m});
            }
        }
    }

    return association_candidates;
}

std::vector<AcceptedAssociation> SlamFrontend::_select_one_to_one_associations(
    std::vector<AssociationCandidate> candidates) const
{
    // Sorting hierarchy:
    // 1. residual ascending
    // 2. valid-detection index ascending
    // 3. target-view index ascending

    std::vector<AcceptedAssociation> accepted_associations;

    if (candidates.empty())
    {
        return accepted_associations;
    }

    accepted_associations.reserve(candidates.size());

    std::set<std::size_t> used_valid_detection_indices;
    std::set<std::size_t> used_target_view_indices;

    std::sort(candidates.begin(), candidates.end(),
              [](const AssociationCandidate& a, const AssociationCandidate& b)
              {
                  return std::tie(a.residual_m, a.valid_detection_index,
                                  a.target_view_index) <
                         std::tie(b.residual_m, b.valid_detection_index,
                                  b.target_view_index);
              });

    for (const AssociationCandidate& candidate : candidates)
    {
        const bool detection_already_used =
            used_valid_detection_indices.count(
                candidate.valid_detection_index) != 0U;

        const bool target_already_used =
            used_target_view_indices.count(candidate.target_view_index) != 0U;

        if (detection_already_used || target_already_used)
        {
            continue;
        }

        accepted_associations.push_back(AcceptedAssociation{
            candidate.valid_detection_index, candidate.target_view_index,
            candidate.residual_m});

        used_valid_detection_indices.insert(candidate.valid_detection_index);
        used_target_view_indices.insert(candidate.target_view_index);
    }

    return accepted_associations;
}

void SlamFrontend::_emit_optimized_associations(
    const std::vector<ValidDetection>& valid_detections,
    const std::vector<PredictedLandmarkMeasurement>& predicted_landmarks,
    const std::vector<AcceptedAssociation>& optimized_associations,
    FrontendResult& result)
{
    for (const AcceptedAssociation& association : optimized_associations)
    {
        const ValidDetection& detection =
            valid_detections.at(association.valid_detection_index);

        const PredictedLandmarkMeasurement& target =
            predicted_landmarks.at(association.target_view_index);

        result.landmark_observations.push_back(LandmarkObservation{
            target.landmark_id, detection.measurement_base_m,
            LandmarkAssociation::ExistingMapLandmark, association.residual_m

        });
    }
}

void SlamFrontend::_apply_tentative_track_associations(
    const ConeFrame& frame,
    const std::vector<ValidDetection>& valid_detections,
    const std::vector<LocalTrackTarget>& tentative_targets,
    const std::vector<AcceptedAssociation>& associations)
{
    if (associations.empty())
    {
        return;
    }

    for (const AcceptedAssociation& association : associations)
    {
        const ValidDetection& detection =
            valid_detections[association.valid_detection_index];
        const LocalTrackTarget& target =
            tentative_targets.at(association.target_view_index);
        LocalLandmarkTrack& local_track =
            _local_tracks.at(target.local_track_index);
        const transforms::Point2D& detection_odom = detection.position_odom_m;

        const double old_x = local_track.position_odom_m.x_m;
        const double old_y = local_track.position_odom_m.y_m;
        const std::uint64_t old_count = local_track.observation_count;

        local_track.position_odom_m.x_m =
            (old_count * old_x + detection_odom.x_m) / (old_count + 1);
        local_track.position_odom_m.y_m =
            (old_count * old_y + detection_odom.y_m) / (old_count + 1);

        local_track.observation_count++;
        local_track.last_seen_ns = frame.timestamp_ns;
    }
}

void SlamFrontend::_apply_pending_track_associations(
    const ConeFrame& frame,
    const std::vector<ValidDetection>& valid_detections,
    const std::vector<LocalTrackTarget>& pending_targets,
    const std::vector<AcceptedAssociation>& associations,
    FrontendResult& result)
{
    if (associations.empty())
    {
        return;
    }

    for (const AcceptedAssociation& association : associations)
    {
        const ValidDetection& detection =
            valid_detections[association.valid_detection_index];
        const LocalTrackTarget& target =
            pending_targets.at(association.target_view_index);
        LocalLandmarkTrack& local_track =
            _local_tracks.at(target.local_track_index);
        const transforms::Point2D& detection_odom = detection.position_odom_m;

        const double old_x = local_track.position_odom_m.x_m;
        const double old_y = local_track.position_odom_m.y_m;
        const std::uint64_t old_count = local_track.observation_count;

        local_track.position_odom_m.x_m =
            (old_count * old_x + detection_odom.x_m) / (old_count + 1);
        local_track.position_odom_m.y_m =
            (old_count * old_y + detection_odom.y_m) / (old_count + 1);

        local_track.observation_count++;
        local_track.last_seen_ns = frame.timestamp_ns;

        if (!local_track.landmark_id.has_value())
        {
            throw std::logic_error("Pending track has no landmark ID");
        }

        result.landmark_observations.push_back(LandmarkObservation{
            *local_track.landmark_id,
            detection.measurement_base_m,
            LandmarkAssociation::PendingLandmark,
            association.residual_m,
        });
    }
}

void SlamFrontend::_create_tentative_tracks(
    const ConeFrame& frame,
    const std::vector<ValidDetection>& valid_detections,
    const std::vector<std::size_t>& unmatched_valid_detection_indices)
{
    if (unmatched_valid_detection_indices.empty())
    {
        return;
    }

    for (const std::size_t valid_detection_index :
         unmatched_valid_detection_indices)
    {
        const transforms::Point2D& position_odom_m =
            valid_detections[valid_detection_index].position_odom_m;

        _local_tracks.push_back(LocalLandmarkTrack{
            _next_local_track_id++,
            LocalTrackState::Tentative,
            std::nullopt,
            position_odom_m,
            1U,
            frame.timestamp_ns,
            frame.timestamp_ns,
        });
    }
}

void SlamFrontend::_promote_eligible_tracks(
    const ConeFrame& frame,
    const std::vector<ValidDetection>& valid_detections,
    const std::vector<LocalTrackTarget>& tentative_targets,
    const std::vector<AcceptedAssociation>& tentative_associations,
    FrontendResult& result)
{
    for (const AcceptedAssociation& association : tentative_associations)
    {
        const ValidDetection& detection =
            valid_detections.at(association.valid_detection_index);

        const LocalTrackTarget& target =
            tentative_targets.at(association.target_view_index);

        LocalLandmarkTrack& local_track =
            _local_tracks.at(target.local_track_index);

        if (local_track.observation_count >=
                _params.minimum_observations_to_confirm &&
            local_track.state == LocalTrackState::Tentative)
        {
            local_track.state = LocalTrackState::Pending;
            local_track.landmark_id = _next_landmark_id;
            local_track.last_seen_ns = frame.timestamp_ns;

            result.timestamp_ns = frame.timestamp_ns;
            result.new_landmark_ids.push_back(*local_track.landmark_id);
            result.debug.tracks_promoted++;
            result.landmark_observations.push_back(LandmarkObservation{
                *local_track.landmark_id,
                detection.measurement_base_m,
                LandmarkAssociation::NewLandmark,
                association.residual_m,
            });

            _next_landmark_id++;
        }
    }
}

void SlamFrontend::_remove_stale_local_tracks(std::int64_t current_timestamp_ns,
                                              FrontendDiagnostics& diagnostics)
{
    const std::size_t num_tracks_before = _local_tracks.size();

    _local_tracks.erase(
        std::remove_if(
            _local_tracks.begin(), _local_tracks.end(),
            [current_timestamp_ns, this](const LocalLandmarkTrack& track)
            {
                if (track.state == LocalTrackState::Tentative)
                {
                    return (current_timestamp_ns - track.last_seen_ns) >
                           _params.tentative_track_max_age_ns;
                }
                else
                {
                    return (current_timestamp_ns - track.last_seen_ns) >
                           _params.pending_track_max_age_ns;
                }
            }),
        _local_tracks.end());

    const std::size_t num_tracks_after = _local_tracks.size();

    diagnostics.tracks_removed_stale = num_tracks_before - num_tracks_after;
}

// only used in test file
std::vector<AcceptedAssociation> SlamFrontend::associate_points_one_to_one(
    const std::vector<transforms::Point2D>& detections,
    const std::vector<transforms::Point2D>& targets, const double gate_m) const
{
    std::vector<ValidDetection> valid_detections;
    valid_detections.reserve(detections.size());

    for (std::size_t detection_index = 0; detection_index < detections.size();
         ++detection_index)
    {
        valid_detections.push_back(ValidDetection{
            detection_index,
            detections[detection_index],
            detections[detection_index],
            1.0,
        });
    }

    std::vector<LocalTrackTarget> local_targets;
    local_targets.reserve(targets.size());

    for (std::size_t target_index = 0; target_index < targets.size();
         ++target_index)
    {
        local_targets.push_back(LocalTrackTarget{
            target_index,
            targets[target_index],
        });
    }

    const std::vector<AssociationCandidate> candidates =
        _build_association_candidates(valid_detections, local_targets, gate_m);

    return _select_one_to_one_associations(candidates);
}

}  // namespace slam::frontend
