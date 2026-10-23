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
 * One perception frame: cone detections plus the odometry pose they were taken at
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
    // Marginal covariance of the latest solved pose (x, y, yaw), in the pose's own frame
    Eigen::Matrix3d pose_covariance = Eigen::Matrix3d::Zero();
    std::vector<MapLandmark> landmarks;
};

enum class LandmarkAssociation { ExistingMapLandmark, PendingLandmark, NewLandmark };

struct LandmarkObservation {
    std::uint64_t landmark_id{};
    Eigen::Vector2d measurement_base_m = Eigen::Vector2d::Zero();
    LandmarkAssociation association{};
};

/**
 * Frontend output and backend input: associated observations at one odometry pose
 */
struct LandmarkFrame {
    std::uint64_t timestamp_ns{};
    Eigen::Isometry2d pose_odom_from_base = Eigen::Isometry2d::Identity();
    std::vector<LandmarkObservation> observations;
};

/**
 * Associates cone detections with the optimized map and with local tracks.
 * A track becomes a landmark once its observations have accumulated enough
 * confidence and is handed to the backend; it is dropped locally once the
 * backend's map contains it.
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
    std::optional<LandmarkFrame> process_frame(const ConeFrame& frame);

    /**
     * Replaces the optimized map used for association.
     *
     * @param state the latest backend map
     * @return false if the map was rejected and nothing changed
     */
    bool update_map_state(const MapState& state);

    /**
     * Optimized landmarks plus landmarks the backend has not acknowledged yet,
     * in map frame, sorted by id, with the landmark fields of each cone set.
     *
     * @return nullptr until the first map state is accepted
     */
    std::shared_ptr<dv_msgs::Cones> planner_map(std::uint64_t timestamp_ns) const;

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

    struct Track {
        // Set once the track is promoted to a landmark
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
    };

    // Associates unused detections with pending or tentative tracks and emits
    // observations for pending tracks and newly promoted ones
    void associate_tracks(bool pending, const std::vector<Detection>& detections,
                          std::vector<bool>& used, LandmarkFrame& out);

    // Chi-squared gate on the Mahalanobis distance between a detection and a map landmark, with
    // the solved pose's covariance in the innovation, so the gate opens along a lap without map
    // matches and closes again at the first one
    double _map_association_chi2{};
    // Detection position noise in the innovation covariance
    double _measurement_std_m{};
    double _local_track_association_gate_m{};
    // Summed detection confidence at which a tentative track becomes a landmark
    double _minimum_evidence_to_confirm{};
    // An observation only adds evidence once the car has moved this far since the last one that
    // did, so a parked car cannot confirm a landmark by staring at it
    double _minimum_evidence_baseline_m{};
    double _minimum_detection_confidence{};

    std::optional<MapState> _map_state;
    std::vector<Track> _tracks;
    std::unordered_map<std::uint64_t, ColorEvidence> _color_by_landmark_id;
    // 0 is the proto default for "no landmark", so ids start at 1
    std::uint64_t _next_landmark_id{1};
    std::optional<std::uint64_t> _last_frame_ns;
    dv_msgs::SlamFrontendDebug _debug;
};

}  // namespace slam
