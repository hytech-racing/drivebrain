#pragma once

#include <Eigen/Core>

#include <atomic>
#include <cstdint>
#include <optional>
#include <vector>

#include <FoxgloveServer.hpp>
#include <StateTracker.hpp>
#include "dv_msgs.pb.h"

namespace planning {

/**
 * Plans along a minimum-curvature racing line once the cone map is a complete, closed track.
 *
 * Until then plan() returns nullopt and the caller keeps using the midline planner (plan_path). That covers
 * lap 1 (the track is still being discovered), open tracks (autocross, which never close), and any map that
 * is too incomplete to build a reliable line from. Nothing here changes what the car does while it returns nullopt.
 *
 * How the line is built (see RacingLine.cpp):
 *   1. order each colour's cones into a closed loop (fails on an open track)
 *   2. build a centreline between the two loops, with how far the track extends left and right at every point
 *   3. choose a sideways offset at every point that minimises how much the path bends, staying a margin inside the cones
 *      (car half width + cone radius + clearance)
 */
class RacingLinePlanner {
public:
  /** Loads the "racing_line" config block and keeps it updated from live param changes; false if any value is missing */
  bool init();

  /**
   * @param cones the SLAM cone map, map frame
   * @param scan_ns time of the scan the map came from
   * @return the line ahead of the car, map frame, or nullopt when the map is not a closed track
   *         (or the planner is disabled, or the car is not pointing along the line)
   */
  std::optional<std::vector<core::xyz_vec<float>>> plan(const dv_msgs::Cones& cones, std::uint64_t scan_ns);

  /** Whether the last plan() found a closed track, i.e. the discovery lap is over (true even when the planner is disabled) */
  bool track_closed() const { return _line.has_value(); }

private:
  /* Set by init() and live param changes (Foxglove thread), read by plan() (autonomy thread). A size change applies at the next replan */
  std::atomic<bool> _enabled{false};
  std::atomic<float> _car_half_width{0.0f};
  std::atomic<float> _cone_radius{0.0f};
  std::atomic<float> _clearance{0.0f};
  boost::signals2::scoped_connection _param_connection;
  std::optional<std::vector<Eigen::Vector2d>> _line;  // closed, in driving direction
  std::optional<std::uint64_t> _last_compute_ns;  // scan time the line was last rebuilt at
  std::optional<bool> _last_reported_ok;  // availability last logged, so only changes are logged
};

}  // namespace planning
