#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include <StateTracker.hpp>
#include "RacingLine.hpp"
#include "dv_msgs.pb.h"

namespace planning {

/**
 * Plans along a minimum-curvature racing line once the cone map is a complete, closed track.
 *
 * Until then plan() returns nullopt and the caller keeps using the midline planner (plan_path). That covers
 * lap 1 (the track is still being discovered), open tracks (autocross, which never close), and any map that
 * is too incomplete to build a reliable line from. Nothing here changes what the car does while it returns nullopt.
 */
class RacingLinePlanner {
public:
  struct Config {
    bool enabled = false;
    racing_line::Params line;
    std::size_t window_points = 30;  // points handed to the controller, one per gate_spacing_m
    double replan_period_s = 1.0;    // the line is rebuilt from the map at most this often
    // Lap 1 is the discovery lap (midline planner, the map is still being built). Then the midline planner is kept for
    // this many laps on the closed map, then the racing line is followed for racing_laps laps. 1 and 1 give one lap of
    // each on the same map to compare; 0 midline laps switches to the racing line as soon as the lap is over.
    int midline_laps_after_closure = 1;
    int racing_laps = 1;
    // When all of those laps are done, finished() turns true so the caller can stop the car and end the run
    bool end_run_when_done = false;
  };

  RacingLinePlanner();
  explicit RacingLinePlanner(Config config);

  /**
   * @param cones the SLAM cone map, map frame
   * @param scan_ns time of the scan the map came from
   * @return the line ahead of the car, map frame, or nullopt when the map is not a closed track
   *         (or the planner is disabled, or the car is not pointing along the line)
   */
  std::optional<std::vector<core::xyz_vec<float>>> plan(const dv_msgs::Cones& cones, std::uint64_t scan_ns);

  /// True from the moment the map first forms a closed track (the discovery lap is over), whether or not the racing line is enabled
  bool track_closed() const { return _ever_closed; }

  /// True once the discovery lap, the midline laps and the racing-line laps are all complete (and end_run_when_done is set)
  bool finished() const { return _config.enabled && _config.end_run_when_done && _finished; }

private:
  Config _config;
  std::optional<std::vector<racing_line::Vec2>> _line;  // closed, in driving direction
  bool _has_computed = false;
  std::uint64_t _last_compute_ns = 0;
  // Laps are counted from where the car started: one is done when it comes back near the start after having been far from it
  std::optional<racing_line::Vec2> _start_position;
  bool _left_start = false;
  int _laps_completed = 0;
  bool _racing_started = false;
  bool _finished = false;
  bool _ever_closed = false;
  std::optional<bool> _last_reported_ok;
  std::uint64_t _last_report_ns = 0;  // while unavailable, the reason is repeated every few seconds so it is not lost in a busy log
};

}  // namespace planning
