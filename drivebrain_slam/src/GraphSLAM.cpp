#include "GraphSLAM.hpp"

#include <spdlog/spdlog.h>

namespace slam {

GraphSlam::~GraphSlam() {
  stop();
}

void GraphSlam::start() {
  if (_running.exchange(true)) {
    return;
  }
  _thread = std::thread(&GraphSlam::_run, this);
  spdlog::info("SLAM started");
}

void GraphSlam::stop() {
  if (!_running.exchange(false)) {
    return;
  }
  if (_thread.joinable()) {
    _thread.join();
  }
  spdlog::info("SLAM stopped");
}

void GraphSlam::_run() {
  auto next_tick = std::chrono::steady_clock::now();

  while (_running) {
    next_tick += _period;

    auto dv = core::StateTracker::instance().dv_state();
    auto vehicle_state = core::StateTracker::instance().vehicle_state().first;

    bool new_factors = false;

    if (dv.cone_observations && dv.cone_observations != _last_observations) {
      _last_observations = dv.cone_observations;
      new_factors |= _add_landmark_factors(*dv.cone_observations);
    }

    new_factors |= _add_odometry_factor(vehicle_state);

    if (new_factors) {
      _optimize();
    }

    std::this_thread::sleep_until(next_tick);
  }
}

bool GraphSlam::_add_landmark_factors(const dv_msgs::Cones& observations) {
  return false;
}

bool GraphSlam::_add_odometry_factor(const core::VehicleState& vehicle_state) {
  return false;
}

void GraphSlam::_optimize() {
  _isam.update(_graph, _initial_estimate);
  _graph.resize(0);
  _initial_estimate.clear();
}

}
