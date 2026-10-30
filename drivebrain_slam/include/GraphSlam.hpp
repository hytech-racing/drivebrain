#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <vector>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <SlamFrontend.hpp>
#include "dv_msgs.pb.h"

namespace slam {

/**
 * Factor graph SLAM. One pose per frame, odom bewteen adjacaent poses, and bearing-range factors for cones.
 */
class GraphSlam {
public:
    GraphSlam();

    /**
     * Adds a frame to the graph and re-optimizes. New observations
     * initialize a landmark; other observations must reference a known one.
     *
     * @param frame associated observations from the frontend
     * @return the optimized map and map <- odom correction on the frames the graph is
     *         solved; nullopt in between, or after a failure (see failed()), after which
     *         every later call returns nullopt.
     */
    std::optional<MapState> update(const ObservationFrame& frame);

    bool failed() const { return _failed; }

    /* Diagnostics of the most recent update(); the cumulative block spans the whole session */
    const dv_msgs::IncrementalGraphSlamDebug& debug() const { return _debug; }

    /* Optimized pose of every frame in the graph, map frame, by frame index */
    std::map<std::size_t, Eigen::Isometry2d> optimized_poses() const;

private:
    // Graph solve rate
    static constexpr std::size_t SOLVE_PER_FRAME = 5;

    gtsam::ISAM2 _isam;
    gtsam::SharedNoiseModel _prior_noise = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(0.01, 0.01, 0.01));
    gtsam::SharedNoiseModel _odom_noise;
    gtsam::SharedNoiseModel _bearing_range_noise;

    bool _failed{};
    std::size_t _pose_count{};
    std::uint64_t _next_sequence{};
    gtsam::Pose2 _previous_pose_odom_from_base;
    // Map-frame estimate of the previous pose, chained from odometry between solves
    gtsam::Pose2 _previous_pose_guess;
    gtsam::NonlinearFactorGraph _pending_factors;
    gtsam::Values _pending_values;
    std::size_t _frames_since_solve{};
    std::set<std::uint64_t> _landmark_ids;
    dv_msgs::IncrementalGraphSlamDebug _debug;
};

/**
 * The full SLAM stack: frontend data association and the factor graph backend
 */
struct Slam {
    /**
     * Runs one scan's cone detections through the frontend and backend
     * @param cones detected cones in os_sensor
     * @param scan_ns host time of the scan
     * @param pose_odom_from_base the car's odometry pose at the scan time
     * @param T_base_lidar os_sensor -> base_link
     * @return the map of cones, or nullptr before the first map exists
    */
    std::shared_ptr<dv_msgs::Cones> update(const dv_msgs::Cones& cones, std::uint64_t scan_ns,
                                           const Eigen::Isometry2d& pose_odom_from_base,
                                           const Eigen::Isometry3d& T_base_lidar);

    /* Draws the landmarks, this frame's factors and (once a second) the full pose graph */
    void render(std::uint64_t scan_ns, const std::optional<ObservationFrame>& observations,
                const std::shared_ptr<dv_msgs::Cones>& map);

    SlamFrontend frontend;
    GraphSlam backend;
    std::optional<MapState> map_state;
    std::uint64_t last_graph_render_ns{};
};

}  // namespace slam
