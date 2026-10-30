#pragma once

#include <cstddef>
#include <memory>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Geometry.hpp>
#include "dv_msgs.pb.h"

namespace slam {

struct ConeDetection {
    Eigen::Vector2d position_base_m = Eigen::Vector2d::Zero();
    double confidence{};
    dv_msgs::Cones::ConeColor color{dv_msgs::Cones::UNKNOWN};
    double color_confidence{};
};

/**
 * One perception frame snapshot: cone detections plus the odometry pose they were taken at
 */
struct ConeFrame {
    std::uint64_t timestamp_ns{};
    Eigen::Isometry2d pose_odom_from_base = Eigen::Isometry2d::Identity();
    std::vector<ConeDetection> detections;
};

struct MapLandmark {
    std::uint64_t landmark_id{};
    Eigen::Vector2d position_map_m = Eigen::Vector2d::Zero();
};

/**
 * Full optimized map from the backend
 */
struct MapState {
    std::uint64_t sequence{};
    std::uint64_t timestamp_ns{};
    Eigen::Isometry2d pose_map_from_odom = Eigen::Isometry2d::Identity();
    std::vector<MapLandmark> landmarks;
};

/* Where the observed landmark lives: the solved map, only the backend's graph so far, or created by this observation */
enum class LandmarkStatus { InMap, InGraph, New };

struct LandmarkObservation {
    std::uint64_t landmark_id{};
    Eigen::Vector2d measurement_base_m = Eigen::Vector2d::Zero();
    LandmarkStatus status{};
};

/**
 * Frontend output and backend input: associated observations at one odometry pose
 */
struct ObservationFrame {
    std::uint64_t timestamp_ns{};
    Eigen::Isometry2d pose_odom_from_base = Eigen::Isometry2d::Identity();
    std::vector<LandmarkObservation> observations;
};

/**
 * Data association for landmark SLAM. A cone is an unconfirmed candidate (frontend only, gathering
 * evidence), then a candidate in the graph (promoted, not yet in a solved map), then a map landmark.
 * Detections are matched against those three in that order of trust; leftovers start new candidates.
 */
class SlamFrontend {
public:
    /* Reads the gates and thresholds from the Slam block of the config */
    SlamFrontend();

    /**
     * Associates one frame of detections.
     *
     * @param frame detections in base_link plus the odometry pose they were taken at
     * @return observations for the backend, or nullopt if the frame was rejected
     */
    std::optional<ObservationFrame> process_frame(const ConeFrame& frame);

    /**
     * Replaces the optimized map used for association.
     *
     * @param state the latest backend map
     * @return false if the map was rejected and nothing changed
     */
    bool set_map(const MapState& state);

    /**
     * Optimized landmarks plus the candidates in the graph the backend has not solved yet,
     * in map frame, sorted by id, with the landmark fields of each cone set.
     *
     * @return nullptr until the first map is accepted
     */
    std::shared_ptr<dv_msgs::Cones> cone_map(std::uint64_t timestamp_ns) const;

    /* Diagnostics of the most recent process_frame() */
    const dv_msgs::SlamFrontendDebug& debug() const { return _debug; }

private:
    // Summed observation confidence per color
    struct ColorEvidence {
        double blue{};
        double yellow{};
        double orange_small{};
        double orange_big{};

        void add(dv_msgs::Cones::ConeColor color, double confidence);

        // Most supported color and its share of the total evidence
        std::pair<dv_msgs::Cones::ConeColor, double> estimate() const;
    };

    struct Detection {
        Eigen::Vector2d position_base_m;
        Eigen::Vector2d position_odom_m;
        double confidence;
        dv_msgs::Cones::ConeColor color;
        double color_confidence;
    };

    enum class CandidateState { Unconfirmed, InGraph };

    // A cone seen but not yet in the optimized map
    struct Candidate {
        // Set on promotion
        std::optional<std::uint64_t> landmark_id;
        Eigen::Vector2d position_odom_m;
        std::size_t observation_count{};
        // Summed detection confidence score. Must be > minimum_evidence_to_confirm
        double evidence{};
        // Car position when evidence was last added
        Eigen::Vector2d evidence_position_odom_m;
        std::uint64_t last_seen_ns{};
        // Only used before promotion; afterwards evidence is keyed by landmark id
        ColorEvidence color_evidence;

        CandidateState state() const { return landmark_id ? CandidateState::InGraph : CandidateState::Unconfirmed; }
    };

    // Mahalanobis gate against the solved map; a match emits an InMap observation
    void _match_mapped_landmarks(const std::vector<Detection>& detections, std::vector<bool>& used,
                                 ObservationFrame& out);
    // Euclidean gate against candidates already in the backend graph; a match emits an InGraph observation
    void _match_graph_landmarks(const std::vector<Detection>& detections, std::vector<bool>& used,
                                ObservationFrame& out);
    // Euclidean gate against unconfirmed candidates; a match adds evidence and emits New once promoted
    void _match_unconfirmed(const std::vector<Detection>& detections, std::vector<bool>& used, ObservationFrame& out);
    // Each unmatched detection starts an unconfirmed candidate
    void _spawn_candidates(const std::vector<Detection>& detections, const std::vector<bool>& used,
                           const ObservationFrame& out);
    // Drops candidates unseen for longer than their state's max age
    void _prune_candidates(std::uint64_t now_ns);

    // Euclidean-gated matching to the candidates in one state; returns (detection, candidate) index pairs
    std::vector<std::pair<std::size_t, std::size_t>> _match_candidates(CandidateState state,
                                                                       const std::vector<Detection>& detections,
                                                                       std::vector<bool>& used, std::uint64_t now_ns);

    // Chi-squared value for the map association gate
    double _map_gate_chi2{};
    // Detection position noise in the innovation covariance
    double _measurement_std_m{};
    // Odometry drift per frame (random walk); accumulated since a landmark was last observed
    Eigen::Matrix3d _odom_drift_per_frame = Eigen::Matrix3d::Zero();
    double _candidate_gate_m{};
    // Summed detection confidence at which an unconfirmed candidate becomes a landmark
    double _minimum_evidence_to_confirm{};
    // An observation only adds evidence once the car has moved this far since the last one that
    // did, so a parked car cannot confirm a landmark by staring at it
    double _minimum_evidence_baseline_m{};
    double _minimum_detection_confidence{};

    std::optional<MapState> _map_state;
    std::vector<Candidate> _candidates;
    std::unordered_map<std::uint64_t, ColorEvidence> _color_by_landmark_id;
    // 0 is the proto default for "no landmark", so ids start at 1
    std::uint64_t _next_landmark_id{1};
    std::optional<std::uint64_t> _last_frame_ns;
    std::uint64_t _frame_count{};
    std::unordered_map<std::uint64_t, std::uint64_t> _last_observed_frame;
    dv_msgs::SlamFrontendDebug _debug;
};

}  // namespace slam
