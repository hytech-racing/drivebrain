#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BearingRangeFactor.h>

#include <StateTracker.hpp>
#include "dv_msgs.pb.h"

namespace slam {

/**
 * Factor graph SLAM.
 * Fuses BearingRangeFactors, BetweenFactors, and IMU Preintegration factors
 */
class GraphSlam {
public:
  ~GraphSlam();

  /**
   * Spawns the SLAM thread
   */
  void start();

  /**
   * Stops and joins the SLAM thread
   */
  void stop();

private:
  /**
   * The SLAM loop: adds new incoming factors, and re-optimizes
   */
  void _run();

  /**
   * Adds bearing range factors for newly observed cones.
   *
   * @param observations the latest cone observations, vehicle frame
   * @return whether any factors were added
   */
  bool _add_landmark_factors(const dv_msgs::Cones& observations);

  /**
   * Adds a between factor from the vehicle's odometry since the last pose node.
   *
   * @param vehicle_state the latest vehicle state
   * @return whether a factor was added
   */
  bool _add_odometry_factor(const core::VehicleState& vehicle_state);

  /**
   * Runs an incremental update and publishes the resulting pose and map
   */
  void _optimize();

  std::thread _thread;
  std::atomic<bool> _running{false};

  gtsam::ISAM2 _isam;
  gtsam::NonlinearFactorGraph _graph;
  gtsam::Values _initial_estimate;
  uint64_t _pose_index = 0;

  std::shared_ptr<const dv_msgs::Cones> _last_observations;

  static constexpr std::chrono::milliseconds _period{5};
};

}
