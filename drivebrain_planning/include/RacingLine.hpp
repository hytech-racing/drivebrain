#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

// Minimum-curvature racing line through a CLOSED track marked by blue (left) and yellow (right) cones.
//
// Pure geometry: no protobuf, no StateTracker, so it can be unit tested on its own.
//
// Pipeline (see compute()):
//   1. order_closed_loop()  unordered cones of one colour -> an ordered closed loop (fails on an open track)
//   2. build_gates()        two loops -> a centreline with, at every point, how far the track extends left and right
//   3. solve_offsets()      choose a sideways offset at every gate that minimises the bending of the path
//                           (a QP: minimise sum |p[i-1] - 2 p[i] + p[i+1]|^2, offsets limited by the cones)

namespace planning::racing_line {

using Vec2 = Eigen::Vector2d;

struct Params {
  double gate_spacing_m = 1.0;      // distance between gates along the centreline
  // How far the car's centre must stay from each cone line = car half width + cone radius + clearance, see margin().
  // The first two are physical sizes of the car and the cones; the clearance is the gap kept between the car and a cone
  // (room for path tracking and SLAM error). Measured in the xcel sim: tracking within ~0.1-0.3 m, so ~0.3 m clearance.
  double car_half_width_m = 0.70;   // half the car's width. Placeholder (xcel sim car): set from the real car
  double cone_radius_m = 0.114;     // radius of a small (blue/yellow) cone base: 228 mm across
  double clearance_m = 0.30;        // gap kept between the car's side and a cone, on top of the sizes above
  /** Distance the car's centre is kept from each cone line */
  double margin() const { return car_half_width_m + cone_radius_m + clearance_m; }
  double max_cone_gap_m = 11.0;     // largest gap between neighbouring cones of one colour on a loop
                                    //   (cones are up to 5 m apart, so this survives ONE missing cone, not two in a row)
  double min_loop_length_m = 30.0;  // anything shorter is not a track
  double min_cone_fraction = 0.85;  // share of a colour's cones that must lie on its loop (rest are treated as strays)
  double max_half_width_m = 4.0;    // a boundary further than this from the centreline is not this gate's boundary
  double regularisation = 1e-4;     // tiny pull towards the centreline where curvature does not care
  int max_iterations = 5000;        // QP solver iteration cap
  double admm_rho = 0.03;           // QP solver step parameter: only changes how fast it converges, not the answer (tuned: 9x fewer iterations than 1.0)
  double tolerance_m = 5e-4;        // QP solver stops when the offsets change by less than this
  int reweight_passes = 4;          // re-solves that correct the cost for uneven point spacing (0 = plain second differences)
};

/** The track as seen from the centreline. normal[i] points left; left[i]/right[i] are distances to the cone lines. */
struct Gates {
  std::vector<Vec2> centre;
  std::vector<Vec2> normal;
  std::vector<double> left;
  std::vector<double> right;
};

/** Orders unordered same-colour cones into a closed loop, or returns nullopt (open track, gap too big, too many strays) */
std::optional<std::vector<Vec2>> order_closed_loop(const std::vector<Vec2>& cones, const Params& params, std::string* error = nullptr);

/** Builds gates from the blue (left) and yellow (right) loops. Centreline runs with blue on its left. */
std::optional<Gates> build_gates(const std::vector<Vec2>& blue_loop, const std::vector<Vec2>& yellow_loop, const Params& params,
                                 std::string* error = nullptr);

/**
 * Sideways offset (m, positive = left) at every gate that minimises curvature, within the cone lines minus the margin.
 * A gate narrower than twice the margin is pinned to the middle of the track.
 */
std::vector<double> solve_offsets(const Gates& gates, const Params& params, int* iterations_used = nullptr);

/** Everything above in one call. The result is a closed line, in driving direction, one point per gate. */
std::optional<std::vector<Vec2>> compute(const std::vector<Vec2>& blue_cones, const std::vector<Vec2>& yellow_cones, const Params& params,
                                         std::string* error = nullptr);

/**
 * The next n points of a closed line, starting at the point closest to pos among those that run the way the car points.
 * Returns nullopt, so the caller falls back to the midline planner, when no such point is within max_distance_m:
 * the car is pointing against the line, or is far from it. A path that starts far away is never better than no path.
 */
std::optional<std::vector<Vec2>> window_ahead(const std::vector<Vec2>& closed_line, const Vec2& pos, const Vec2& heading, std::size_t n_points,
                                              double max_distance_m = 5.0);

/** Signed curvature (1/m) at each point of a closed path, three-point circle formula */
std::vector<double> curvature(const std::vector<Vec2>& closed_path);

}  // namespace planning::racing_line
