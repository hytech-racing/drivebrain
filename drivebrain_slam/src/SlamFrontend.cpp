#include "SlamFrontend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <tuple>
#include <unordered_set>

#include <FoxgloveServer.hpp>

namespace slam {

namespace {

struct Candidate {
    double distance;
    std::size_t detection;
    std::size_t target;
};

/**
 * Greedy one-to-one matching of unused detections to targets, closest pairs first
 *
 * @param distance distance(d, t) between detection d and target t, compared against the gate
 * @return the (detection, target) pairs matched; matched detections are marked used
 */
template <class Distance>
std::vector<std::pair<std::size_t, std::size_t>> match_nearest_first(const std::size_t num_detections,
                                                                     std::vector<bool>& used,
                                                                     const std::size_t num_targets,
                                                                     const Distance& distance, const double gate) {
    std::vector<Candidate> candidates;
    for (std::size_t d = 0; d < num_detections; ++d) {
        if (used[d]) {
            continue;
        }
        for (std::size_t t = 0; t < num_targets; ++t) {
            const double value = distance(d, t);
            if (value <= gate) {
                candidates.push_back({value, d, t});
            }
        }
    }

    std::sort(candidates.begin(), candidates.end(), [](const Candidate& a, const Candidate& b) {
        return std::tie(a.distance, a.detection, a.target) <
               std::tie(b.distance, b.detection, b.target);
    });

    std::vector<bool> target_used(num_targets, false);
    std::vector<std::pair<std::size_t, std::size_t>> matches;
    for (const Candidate& candidate : candidates) {
        if (used[candidate.detection] || target_used[candidate.target]) {
            continue;
        }
        used[candidate.detection] = true;
        target_used[candidate.target] = true;
        matches.emplace_back(candidate.detection, candidate.target);
    }
    return matches;
}

// A track unseen for this long is dropped
constexpr std::uint64_t TENTATIVE_TRACK_MAX_AGE_NS = 3'000'000'000;
constexpr std::uint64_t PENDING_TRACK_MAX_AGE_NS = 1'000'000'000;

}  // namespace

SlamFrontend::SlamFrontend() {
    _map_association_chi2 = core::FoxgloveServer::instance().get_param<float>("Slam/map_association_chi2").value();
    _measurement_std_m = core::FoxgloveServer::instance().get_param<float>("Slam/measurement_std_m").value();
    _local_track_association_gate_m =
        core::FoxgloveServer::instance().get_param<float>("Slam/local_track_association_gate_m").value();
    _minimum_evidence_to_confirm =
        core::FoxgloveServer::instance().get_param<float>("Slam/minimum_evidence_to_confirm").value();
    _minimum_evidence_baseline_m =
        core::FoxgloveServer::instance().get_param<float>("Slam/minimum_evidence_baseline_m").value();
    _minimum_detection_confidence =
        core::FoxgloveServer::instance().get_param<float>("Slam/minimum_detection_confidence").value();
}

std::optional<LandmarkFrame> SlamFrontend::process_frame(const ConeFrame& frame) {
    _debug.Clear();
    _debug.set_timestamp_ns(frame.timestamp_ns);
    dv_msgs::SlamFrontendDiagnostics& diag = *_debug.mutable_diagnostics();
    diag.set_detections_received(frame.detections.size());
    if ((_last_frame_ns && frame.timestamp_ns <= *_last_frame_ns) ||
        !frame.pose_odom_from_base.matrix().allFinite()) {
        return std::nullopt;
    }
    _last_frame_ns = frame.timestamp_ns;

    std::vector<Detection> detections;
    for (const ConeDetection& detection : frame.detections) {
        if (detection.position_base_m.allFinite() && std::isfinite(detection.confidence) &&
            detection.confidence <= 1.0 &&
            detection.confidence >= _minimum_detection_confidence) {
            detections.push_back({detection.position_base_m,
                                  frame.pose_odom_from_base * detection.position_base_m,
                                  detection.confidence, detection.color, detection.color_confidence});
        } else {
            diag.set_detections_rejected_invalid(diag.detections_rejected_invalid() + 1);
        }
    }
    _debug.set_frame_accepted(true);
    std::vector<bool> used(detections.size(), false);

    LandmarkFrame out{frame.timestamp_ns, frame.pose_odom_from_base, {}};

    if (_map_state && _map_state->timestamp_ns <= frame.timestamp_ns) {
        const Eigen::Isometry2d pose_odom_from_map = _map_state->pose_map_from_odom.inverse();
        std::vector<std::size_t> target_indices;
        std::vector<Eigen::Vector2d> targets_odom;
        for (std::size_t i = 0; i < _map_state->landmarks.size(); ++i) {
            target_indices.push_back(i);
            targets_odom.push_back(pose_odom_from_map * _map_state->landmarks[i].position_map_m);
        }

        // Innovation covariance: the solved pose's uncertainty (GTSAM expresses it in the pose's
        // own frame) rotated into odom and projected through the lever arm to the landmark, plus
        // the detection noise
        Eigen::Matrix3d odom_from_body = Eigen::Matrix3d::Identity();
        odom_from_body.topLeftCorner<2, 2>() = frame.pose_odom_from_base.rotation();
        const Eigen::Matrix3d pose_covariance_odom = odom_from_body * _map_state->pose_covariance * odom_from_body.transpose();
        const Eigen::Vector2d car_odom_m = frame.pose_odom_from_base.translation();
        const Eigen::Matrix2d measurement_covariance =
            _measurement_std_m * _measurement_std_m * Eigen::Matrix2d::Identity();
        const auto mahalanobis_squared = [&](std::size_t d, std::size_t t) {
            const Eigen::Vector2d lever = targets_odom[t] - car_odom_m;
            Eigen::Matrix<double, 2, 3> jacobian;
            jacobian << 1.0, 0.0, -lever.y(), 0.0, 1.0, lever.x();
            const Eigen::Matrix2d S = jacobian * pose_covariance_odom * jacobian.transpose() + measurement_covariance;
            const Eigen::Vector2d innovation = detections[d].position_odom_m - targets_odom[t];
            return innovation.dot(S.ldlt().solve(innovation));
        };

        for (const auto& [d, t] : match_nearest_first(detections.size(), used, targets_odom.size(),
                                                      mahalanobis_squared, _map_association_chi2)) {
            const std::uint64_t id = _map_state->landmarks[target_indices[t]].landmark_id;
            out.observations.push_back(
                {id, detections[d].position_base_m, LandmarkAssociation::ExistingMapLandmark});
            _color_by_landmark_id[id].add(detections[d].color, detections[d].color_confidence);
            diag.set_optimized_associations(diag.optimized_associations() + 1);
        }
    }

    associate_tracks(true, detections, used, out);
    associate_tracks(false, detections, used, out);

    for (std::size_t d = 0; d < detections.size(); ++d) {
        if (used[d]) {
            continue;
        }
        Track track{std::nullopt, detections[d].position_odom_m, 1, detections[d].confidence,
                    frame.pose_odom_from_base.translation(), frame.timestamp_ns, {}};
        track.color_evidence.add(detections[d].color, detections[d].color_confidence);
        _tracks.push_back(track);
        diag.set_tentative_tracks_created(diag.tentative_tracks_created() + 1);
    }

    const std::size_t before = _tracks.size();
    _tracks.erase(std::remove_if(_tracks.begin(), _tracks.end(),
                                 [&](const Track& track) {
                                     const std::uint64_t max_age_ns =
                                         track.landmark_id ? PENDING_TRACK_MAX_AGE_NS : TENTATIVE_TRACK_MAX_AGE_NS;
                                     return frame.timestamp_ns - track.last_seen_ns > max_age_ns;
                                 }),
                  _tracks.end());
    diag.set_tracks_removed_stale(before - _tracks.size());

    _debug.set_landmark_observations(out.observations.size());
    for (const LandmarkObservation& observation : out.observations) {
        if (observation.association == LandmarkAssociation::NewLandmark) {
            _debug.add_new_landmark_ids(observation.landmark_id);
        }
    }

    return out;
}

bool SlamFrontend::update_map_state(const MapState& state) {
    if (!state.pose_map_from_odom.matrix().allFinite() ||
        (_map_state && (state.timestamp_ns <= _map_state->timestamp_ns ||
                        state.sequence <= _map_state->sequence))) {
        return false;
    }

    std::unordered_set<std::uint64_t> map_ids;
    for (const MapLandmark& landmark : state.landmarks) {
        if (!landmark.position_map_m.allFinite() ||
            landmark.landmark_id == std::numeric_limits<std::uint64_t>::max() ||
            !map_ids.insert(landmark.landmark_id).second) {
            return false;
        }
    }

    for (const std::uint64_t id : map_ids) {
        _next_landmark_id = std::max(_next_landmark_id, id + 1);
    }

    _tracks.erase(std::remove_if(_tracks.begin(), _tracks.end(),
                                 [&](const Track& track) {
                                     return track.landmark_id && map_ids.count(*track.landmark_id);
                                 }),
                  _tracks.end());

    _map_state = state;
    return true;
}

std::shared_ptr<dv_msgs::Cones> SlamFrontend::planner_map(const std::uint64_t timestamp_ns) const {
    if (!_map_state) {
        return nullptr;
    }

    auto map = std::make_shared<dv_msgs::Cones>();
    map->set_timestamp_us(timestamp_ns / 1000);

    const auto add = [&](std::uint64_t id, const Eigen::Vector2d& position_map_m, bool optimized) {
        dv_msgs::Cones::Cone* cone = map->add_cones();
        cone->mutable_position()->set_x(static_cast<float>(position_map_m.x()));
        cone->mutable_position()->set_y(static_cast<float>(position_map_m.y()));
        cone->set_landmark_id(id);
        cone->set_optimized(optimized);
        const auto it = _color_by_landmark_id.find(id);
        if (it != _color_by_landmark_id.end()) {
            const auto [color, confidence] = it->second.estimate();
            cone->set_color(color);
            cone->set_color_confidence(static_cast<float>(confidence));
        }
    };

    for (const MapLandmark& landmark : _map_state->landmarks) {
        add(landmark.landmark_id, landmark.position_map_m, true);
    }
    for (const Track& track : _tracks) {
        if (track.landmark_id) {
            add(*track.landmark_id, _map_state->pose_map_from_odom * track.position_odom_m, false);
        }
    }

    std::sort(map->mutable_cones()->begin(), map->mutable_cones()->end(),
              [](const dv_msgs::Cones::Cone& a, const dv_msgs::Cones::Cone& b) {
                  return a.landmark_id() < b.landmark_id();
              });
    return map;
}

void SlamFrontend::associate_tracks(const bool pending, const std::vector<Detection>& detections,
                                     std::vector<bool>& used, LandmarkFrame& out) {
    dv_msgs::SlamFrontendDiagnostics& diag = *_debug.mutable_diagnostics();
    std::vector<std::size_t> track_indices;
    std::vector<Eigen::Vector2d> targets_odom;
    for (std::size_t i = 0; i < _tracks.size(); ++i) {
        if (_tracks[i].landmark_id.has_value() == pending) {
            track_indices.push_back(i);
            targets_odom.push_back(_tracks[i].position_odom_m);
        }
    }

    const auto euclidean = [&](std::size_t d, std::size_t t) {
        return (detections[d].position_odom_m - targets_odom[t]).norm();
    };
    for (const auto& [d, t] : match_nearest_first(detections.size(), used, targets_odom.size(), euclidean,
                                                  _local_track_association_gate_m)) {
        const Detection& detection = detections[d];
        Track& track = _tracks[track_indices[t]];

        const double n = static_cast<double>(track.observation_count);
        track.position_odom_m = (n * track.position_odom_m + detection.position_odom_m) / (n + 1.0);
        track.observation_count++;
        track.last_seen_ns = out.timestamp_ns;

        if (pending) {
            _color_by_landmark_id[*track.landmark_id].add(detection.color,
                                                          detection.color_confidence);
            out.observations.push_back(
                {*track.landmark_id, detection.position_base_m, LandmarkAssociation::PendingLandmark});
            diag.set_pending_associations(diag.pending_associations() + 1);
            continue;
        }

        diag.set_tentative_associations(diag.tentative_associations() + 1);
        track.color_evidence.add(detection.color, detection.color_confidence);
        const Eigen::Vector2d car_odom_m = out.pose_odom_from_base.translation();
        if ((car_odom_m - track.evidence_position_odom_m).norm() >= _minimum_evidence_baseline_m) {
            track.evidence += detection.confidence;
            track.evidence_position_odom_m = car_odom_m;
        }
        if (track.evidence >= _minimum_evidence_to_confirm) {
            diag.set_tracks_promoted(diag.tracks_promoted() + 1);
            track.landmark_id = _next_landmark_id++;
            _color_by_landmark_id[*track.landmark_id] = track.color_evidence;
            out.observations.push_back(
                {*track.landmark_id, detection.position_base_m, LandmarkAssociation::NewLandmark});
        }
    }
}

void SlamFrontend::ColorEvidence::add(const dv_msgs::Cones::ConeColor color, const double confidence) {
    if (!std::isfinite(confidence) || confidence <= 0.0) {
        return;
    }
    switch (color) {
        case dv_msgs::Cones::BLUE: blue += confidence; break;
        case dv_msgs::Cones::YELLOW: yellow += confidence; break;
        case dv_msgs::Cones::ORANGE_SMALL: orange_small += confidence; break;
        case dv_msgs::Cones::ORANGE_BIG: orange_big += confidence; break;
        default: break;
    }
}

std::pair<dv_msgs::Cones::ConeColor, double> SlamFrontend::ColorEvidence::estimate() const {
    const double total = blue + yellow + orange_small + orange_big;
    if (!std::isfinite(total) || total <= 0.0) {
        return {dv_msgs::Cones::UNKNOWN, 0.0};
    }

    std::pair<dv_msgs::Cones::ConeColor, double> best{dv_msgs::Cones::BLUE, blue};
    for (const std::pair<dv_msgs::Cones::ConeColor, double>& candidate :
         {std::pair{dv_msgs::Cones::YELLOW, yellow}, std::pair{dv_msgs::Cones::ORANGE_SMALL, orange_small},
          std::pair{dv_msgs::Cones::ORANGE_BIG, orange_big}}) {
        if (candidate.second > best.second) {
            best = candidate;
        }
    }
    return {best.first, best.second / total};
}

}  // namespace slam
