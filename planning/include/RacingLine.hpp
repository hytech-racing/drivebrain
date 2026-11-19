#pragma once

#include <Eigen/Core>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

#include <FoxgloveServer.hpp>
#include <StateTracker.hpp>
#include "dv_msgs.pb.h"

namespace planning {

/**
 * Minimum-curvature racing line, rebuilt from the SLAM map on its own thread at replan_hz.
 * plan() returns nullopt until the map is a closed track, so the caller stays on the midline planner.
 */
class RacingLinePlanner {
public:
  ~RacingLinePlanner();

  /** Loads the "racing_line" config block and starts the rebuild thread; false if any value is missing */
  bool init();

  /** The line ahead of the car, map frame, or nullopt when the map is not closed, the planner is disabled, or the car is off the line */
  std::optional<std::vector<core::xyz_vec<float>>> plan() const;

  bool enabled() const { return _enabled; }

  /** Whether the map is a closed track, i.e. the discovery lap is over (true even when the planner is disabled) */
  bool track_closed() const { return _latest_line() != nullptr; }

private:
  using Line = std::vector<Eigen::Vector2d>;  // closed, in driving direction

  void _run();
  std::shared_ptr<const Line> _latest_line() const;

  std::atomic<bool> _enabled{false};
  std::atomic<float> _car_half_width{0.0f};
  std::atomic<float> _cone_radius{0.0f};
  std::atomic<float> _clearance{0.0f};
  std::atomic<float> _replan_hz{0.0f};
  boost::signals2::scoped_connection _param_connection;

  mutable std::mutex _line_mutex;
  std::shared_ptr<const Line> _line;  // nullptr while the map is not a closed track

  std::thread _thread;
  std::mutex _stop_mutex;
  std::condition_variable _stop_signal;
  bool _running = false;  // guarded by _stop_mutex
};

}  // namespace planning
