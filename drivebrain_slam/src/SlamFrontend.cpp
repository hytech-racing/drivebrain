#include "SlamFrontend.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <unordered_set>

#include <FoxgloveServer.hpp>

namespace slam {

namespace {

struct Pair {
    double distance;
    std::size_t detection;
    std::size_t target;
};

/**
 * Greedy one-to-one matching of unused detections to targets, closest pairs first
 *
 * @param distance distance(d, t) between detection d (row) and target t (column), compared against the gate
 * @return the (detection, target) pairs matched; matched detections are marked used
 */
std::vector<std::pair<std::size_t, std::size_t>> greedy_nearest_neighbor(const Eigen::MatrixXd& distance,
                                                                         std::vector<bool>& used, const double gate) {
    std::vector<Pair> pairs;
    for (std::size_t d = 0; d < static_cast<std::size_t>(distance.rows()); ++d) {
        if (used[d]) {
            continue;
        }
        for (std::size_t t = 0; t < static_cast<std::size_t>(distance.cols()); ++t) {
            if (distance(d, t) <= gate) {
                pairs.push_back({distance(d, t), d, t});
            }
        }
    }

    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) {
        return std::tie(a.distance, a.detection, a.target) < std::tie(b.distance, b.detection, b.target);
    });

    std::vector<bool> target_used(distance.cols(), false);
    std::vector<std::pair<std::size_t, std::size_t>> matches;
    for (const Pair& pair : pairs) {
        if (used[pair.detection] || target_used[pair.target]) {
            continue;
        }
        used[pair.detection] = true;
        target_used[pair.target] = true;
        matches.emplace_back(pair.detection, pair.target);
    }
    return matches;
}

/**
 * Squared Mahalanobis distance of a detection from a map landmark in the odom frame
 */
double mahalanobis_squared(const Eigen::Vector2d& detection_odom, const Eigen::Vector2d& landmark_odom,
                           const Eigen::Vector2d& car_odom, const Eigen::Matrix3d& pose_covariance_odom,
                           const Eigen::Matrix2d& detection_covariance) {
    const Eigen::Vector2d lever = landmark_odom - car_odom;
    Eigen::Matrix<double, 2, 3> J;
    J << 1.0, 0.0, -lever.y(), 0.0, 1.0, lever.x();
    const Eigen::Matrix2d S = J * pose_covariance_odom * J.transpose() + detection_covariance;
    const Eigen::Vector2d innovation = detection_odom - landmark_odom;
    return innovation.dot(S.ldlt().solve(innovation));
}

// A candidate unseen for this long is dropped
constexpr std::uint64_t UNCONFIRMED_MAX_AGE_NS = 3'000'000'000;
constexpr std::uint64_t PROMOTED_MAX_AGE_NS = 1'000'000'000;

}  // namespace

SlamFrontend::SlamFrontend() {
    _map_gate_chi2 = core::FoxgloveServer::instance().get_param<float>("Slam/map_association_gate_chi2").value();
    _measurement_std_m = core::FoxgloveServer::instance().get_param<float>("Slam/measurement_std_m").value();
    _candidate_gate_m = core::FoxgloveServer::instance().get_param<float>("Slam/candidate_gate_m").value();
    _minimum_evidence_to_confirm =
        core::FoxgloveServer::instance().get_param<float>("Slam/minimum_evidence_to_confirm").value();
    _minimum_evidence_baseline_m =
        core::FoxgloveServer::instance().get_param<float>("Slam/minimum_evidence_baseline_m").value();
    _minimum_detection_confidence =
        core::FoxgloveServer::instance().get_param<float>("Slam/minimum_detection_confidence").value();
}

std::optional<ObservationFrame> SlamFrontend::process_frame(const ConeFrame& frame) {
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

    ObservationFrame out{frame.timestamp_ns, frame.pose_odom_from_base, {}};

    // Priority order: the optimized map, then candidates the backend already holds, then unconfirmed ones
    _match_mapped_landmarks(detections, used, out);
    _match_candidates(CandidateState::Promoted, detections, used, out);
    _match_candidates(CandidateState::Unconfirmed, detections, used, out);
    _spawn_candidates(detections, used, frame);
    _prune_candidates(frame.timestamp_ns);

    _debug.set_landmark_observations(out.observations.size());
    for (const LandmarkObservation& observation : out.observations) {
        if (observation.status == LandmarkStatus::New) {
            _debug.add_new_landmark_ids(observation.landmark_id);
        }
    }

    return out;
}

bool SlamFrontend::set_map(const MapState& state) {
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

    _candidates.erase(std::remove_if(_candidates.begin(), _candidates.end(),
                                     [&](const Candidate& candidate) {
                                         return candidate.landmark_id && map_ids.count(*candidate.landmark_id);
                                     }),
                      _candidates.end());

    _map_state = state;
    return true;
}

std::shared_ptr<dv_msgs::Cones> SlamFrontend::cone_map(const std::uint64_t timestamp_ns) const {
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
    for (const Candidate& candidate : _candidates) {
        if (candidate.landmark_id) {
            add(*candidate.landmark_id, _map_state->pose_map_from_odom * candidate.position_odom_m, false);
        }
    }

    std::sort(map->mutable_cones()->begin(), map->mutable_cones()->end(),
              [](const dv_msgs::Cones::Cone& a, const dv_msgs::Cones::Cone& b) {
                  return a.landmark_id() < b.landmark_id();
              });
    return map;
}

void SlamFrontend::_match_mapped_landmarks(const std::vector<Detection>& detections, std::vector<bool>& used,
                                           ObservationFrame& out) {
    if (!_map_state || _map_state->timestamp_ns > out.timestamp_ns) {
        return;
    }
    dv_msgs::SlamFrontendDiagnostics& diag = *_debug.mutable_diagnostics();

    const Eigen::Isometry2d pose_odom_from_map = _map_state->pose_map_from_odom.inverse();
    std::vector<Eigen::Vector2d> landmarks_odom;
    for (const MapLandmark& landmark : _map_state->landmarks) {
        landmarks_odom.push_back(pose_odom_from_map * landmark.position_map_m);
    }

    // GTSAM gives the pose covariance in the pose's own frame; rotate it into odom
    Eigen::Matrix3d odom_from_body = Eigen::Matrix3d::Identity();
    odom_from_body.topLeftCorner<2, 2>() = out.pose_odom_from_base.rotation();
    const Eigen::Matrix3d pose_covariance_odom = odom_from_body * _map_state->pose_covariance * odom_from_body.transpose();
    const Eigen::Vector2d car_odom = out.pose_odom_from_base.translation();
    const Eigen::Matrix2d detection_covariance = _measurement_std_m * _measurement_std_m * Eigen::Matrix2d::Identity();

    Eigen::MatrixXd distance(detections.size(), landmarks_odom.size());
    for (std::size_t d = 0; d < detections.size(); ++d) {
        for (std::size_t l = 0; l < landmarks_odom.size(); ++l) {
            distance(d, l) = mahalanobis_squared(detections[d].position_odom_m, landmarks_odom[l], car_odom,
                                                 pose_covariance_odom, detection_covariance);
        }
    }
    for (const auto& [d, l] : greedy_nearest_neighbor(distance, used, _map_gate_chi2)) {
        const std::uint64_t id = _map_state->landmarks[l].landmark_id;
        out.observations.push_back({id, detections[d].position_base_m, LandmarkStatus::InMap});
        _color_by_landmark_id[id].add(detections[d].color, detections[d].color_confidence);
        diag.set_optimized_associations(diag.optimized_associations() + 1);
    }
}

void SlamFrontend::_match_candidates(const CandidateState state, const std::vector<Detection>& detections,
                                     std::vector<bool>& used, ObservationFrame& out) {
    dv_msgs::SlamFrontendDiagnostics& diag = *_debug.mutable_diagnostics();
    std::vector<std::size_t> candidate_indices;
    std::vector<Eigen::Vector2d> targets_odom;
    for (std::size_t i = 0; i < _candidates.size(); ++i) {
        if (_candidates[i].state() == state) {
            candidate_indices.push_back(i);
            targets_odom.push_back(_candidates[i].position_odom_m);
        }
    }

    Eigen::MatrixXd distance(detections.size(), targets_odom.size());
    for (std::size_t d = 0; d < detections.size(); ++d) {
        for (std::size_t t = 0; t < targets_odom.size(); ++t) {
            distance(d, t) = (detections[d].position_odom_m - targets_odom[t]).norm();
        }
    }
    for (const auto& [d, t] : greedy_nearest_neighbor(distance, used, _candidate_gate_m)) {
        const Detection& detection = detections[d];
        Candidate& candidate = _candidates[candidate_indices[t]];

        const double n = static_cast<double>(candidate.observation_count);
        candidate.position_odom_m = (n * candidate.position_odom_m + detection.position_odom_m) / (n + 1.0);
        candidate.observation_count++;
        candidate.last_seen_ns = out.timestamp_ns;

        if (state == CandidateState::Promoted) {
            _color_by_landmark_id[*candidate.landmark_id].add(detection.color, detection.color_confidence);
            out.observations.push_back(
                {*candidate.landmark_id, detection.position_base_m, LandmarkStatus::InGraph});
            diag.set_pending_associations(diag.pending_associations() + 1);
            continue;
        }

        diag.set_tentative_associations(diag.tentative_associations() + 1);
        candidate.color_evidence.add(detection.color, detection.color_confidence);
        const Eigen::Vector2d car_odom_m = out.pose_odom_from_base.translation();
        if ((car_odom_m - candidate.evidence_position_odom_m).norm() >= _minimum_evidence_baseline_m) {
            candidate.evidence += detection.confidence;
            candidate.evidence_position_odom_m = car_odom_m;
        }
        if (candidate.evidence >= _minimum_evidence_to_confirm) {
            diag.set_tracks_promoted(diag.tracks_promoted() + 1);
            candidate.landmark_id = _next_landmark_id++;
            _color_by_landmark_id[*candidate.landmark_id] = candidate.color_evidence;
            out.observations.push_back({*candidate.landmark_id, detection.position_base_m, LandmarkStatus::New});
        }
    }
}

void SlamFrontend::_spawn_candidates(const std::vector<Detection>& detections, const std::vector<bool>& used,
                                     const ConeFrame& frame) {
    dv_msgs::SlamFrontendDiagnostics& diag = *_debug.mutable_diagnostics();
    for (std::size_t d = 0; d < detections.size(); ++d) {
        if (used[d]) {
            continue;
        }
        Candidate candidate{std::nullopt, detections[d].position_odom_m, 1, detections[d].confidence,
                            frame.pose_odom_from_base.translation(), frame.timestamp_ns, {}};
        candidate.color_evidence.add(detections[d].color, detections[d].color_confidence);
        _candidates.push_back(candidate);
        diag.set_tentative_tracks_created(diag.tentative_tracks_created() + 1);
    }
}

void SlamFrontend::_prune_candidates(const std::uint64_t now_ns) {
    const std::size_t before = _candidates.size();
    _candidates.erase(std::remove_if(_candidates.begin(), _candidates.end(),
                                     [&](const Candidate& candidate) {
                                         const std::uint64_t max_age_ns = candidate.state() == CandidateState::Promoted
                                                                              ? PROMOTED_MAX_AGE_NS
                                                                              : UNCONFIRMED_MAX_AGE_NS;
                                         return now_ns - candidate.last_seen_ns > max_age_ns;
                                     }),
                      _candidates.end());
    _debug.mutable_diagnostics()->set_tracks_removed_stale(before - _candidates.size());
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
