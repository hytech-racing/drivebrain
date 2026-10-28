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
    // Laps to keep following the midline after the track closes, before switching to the racing line. 1 gives a
    // midline lap and a racing-line lap on the same closed map to compare; 0 switches as soon as the line exists.
    int midline_laps_after_closure = 1;
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

private:
  Config _config;
  std::optional<std::vector<racing_line::Vec2>> _line;  // closed, in driving direction
  bool _has_computed = false;
  std::uint64_t _last_compute_ns = 0;
  double _distance_since_closure_m = 0.0;  // driven while the track has been closed; the midline is held until this reaches the laps
  std::optional<racing_line::Vec2> _last_position;
  bool _racing_started = false;
  std::optional<bool> _last_reported_ok;
  std::uint64_t _last_report_ns = 0;  // while unavailable, the reason is repeated every few seconds so it is not lost in a busy log
};

}  // namespace planning
