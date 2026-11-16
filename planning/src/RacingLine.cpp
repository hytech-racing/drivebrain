#include "RacingLine.hpp"

#include <FoxgloveServer.hpp>

#include <Eigen/Sparse>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

namespace planning {

namespace {

using Vec2 = Eigen::Vector2d;

constexpr double GATE_SPACING_M = 1.0;         // distance between gates (points of the line) along the centreline
constexpr double MAX_CONE_GAP_M = 11.0;        // largest gap between neighbouring cones of one colour on a loop
                                               //   (cones are up to 5 m apart, so this survives ONE missing cone, not two in a row)
constexpr double MIN_CONE_FRACTION = 0.85;     // share of a colour's cones that must lie on its loop (rest are treated as strays)
constexpr double MAX_HALF_WIDTH_M = 4.0;       // a boundary further than this from the centreline is not this gate's boundary
constexpr double DENSE_STEP_M = 0.25;          // spacing of the finely sampled cone lines used to measure the track
constexpr double REGULARISATION = 1e-4;        // tiny pull towards the centreline where curvature does not care
constexpr int MAX_ITERATIONS = 5000;           // QP solver iteration cap
constexpr double ADMM_RHO = 0.03;              // QP solver step parameter: only changes how fast it converges, not the answer
constexpr double TOLERANCE_M = 5e-4;           // QP solver stops when the offsets change by less than this
constexpr std::size_t WINDOW_POINTS = 30;      // points handed to the controller, one per GATE_SPACING_M
constexpr double MAX_WINDOW_DISTANCE_M = 5.0;  // the line must pass within this of the car to be followed
constexpr double REPLAN_PERIOD_S = 1.0;        // the line is rebuilt from the map at most this often

/** The track as seen from the centreline. normal[i] points left; left[i]/right[i] are distances to the cone lines. */
struct Gates {
  std::vector<Vec2> centre;
  std::vector<Vec2> normal;
  std::vector<double> left;
  std::vector<double> right;
};

/** The closed loop with straight lines between its points, sampled about every `step` metres */
std::vector<Vec2> densify_closed(const std::vector<Vec2>& loop, double step) {
  const std::size_t n = loop.size();
  std::vector<Vec2> out;
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2& a = loop[i];
    const Vec2& b = loop[(i + 1) % n];
    const int k = std::max(1, static_cast<int>(std::ceil((b - a).norm() / step)));
    for (int s = 0; s < k; ++s) out.push_back(a + (b - a) * (static_cast<double>(s) / k));
  }
  return out;
}

/** Points evenly spaced by arc length (about `spacing` m) around a closed path */
std::vector<Vec2> resample_closed(const std::vector<Vec2>& path, double spacing) {
  const std::size_t n = path.size();
  std::vector<double> s(n + 1, 0.0);
  for (std::size_t i = 0; i < n; ++i) s[i + 1] = s[i] + (path[(i + 1) % n] - path[i]).norm();
  const double total = s[n];
  const std::size_t count = std::max<std::size_t>(8, static_cast<std::size_t>(std::lround(total / spacing)));
  std::vector<Vec2> out;
  out.reserve(count);
  std::size_t seg = 0;
  for (std::size_t k = 0; k < count; ++k) {
    const double target = total * static_cast<double>(k) / static_cast<double>(count);
    while (seg + 1 < n && s[seg + 1] < target) ++seg;
    const double len = s[seg + 1] - s[seg];
    const double f = len > 1e-9 ? (target - s[seg]) / len : 0.0;
    out.push_back(path[seg] + f * (path[(seg + 1) % n] - path[seg]));
  }
  return out;
}

std::vector<Vec2> tangents(const std::vector<Vec2>& p) {
  const std::size_t n = p.size();
  std::vector<Vec2> t(n);
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 d = p[(i + 1) % n] - p[(i + n - 1) % n];
    t[i] = d.norm() > 1e-9 ? Vec2(d.normalized()) : Vec2(1.0, 0.0);
  }
  return t;
}

Vec2 left_of(const Vec2& t) { return {-t.y(), t.x()}; }

std::size_t nearest(const std::vector<Vec2>& points, const Vec2& q) {
  std::size_t best = 0;
  double best_d = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double d = (points[i] - q).squaredNorm();
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  return best;
}

/** Orders unordered same-colour cones into a closed loop, or returns nullopt with the reason (open track, gap too big, too many strays) */
std::optional<std::vector<Vec2>> order_closed_loop(const std::vector<Vec2>& cones, std::string& error) {
  const std::size_t n = cones.size();
  if (n < 6) {
    error = "fewer than 6 cones";
    return std::nullopt;
  }

  // Walk from cone to cone: always the nearest unvisited cone within the gap limit that is not behind the walk direction
  std::vector<char> used(n, 0);
  std::vector<Vec2> loop{cones[0]};
  used[0] = 1;
  std::size_t current = 0;
  Vec2 previous_step = Vec2::Zero();
  bool have_step = false;

  while (true) {
    std::size_t best = n;
    double best_cost = std::numeric_limits<double>::max();
    for (std::size_t j = 0; j < n; ++j) {
      if (used[j]) continue;
      const Vec2 step = cones[j] - cones[current];
      const double d = step.norm();
      if (d > MAX_CONE_GAP_M) continue;
      double cost = d;
      if (have_step) {
        const double cos_turn = step.dot(previous_step) / (d * previous_step.norm());
        if (cos_turn <= 0.0) continue;  // never double back more than 90 degrees
        cost = d * (2.0 - cos_turn);    // prefer straight on over a sharp turn
      }
      if (cost < best_cost) {
        best_cost = cost;
        best = j;
      }
    }
    if (best == n) break;
    previous_step = cones[best] - cones[current];
    have_step = true;
    current = best;
    used[current] = 1;
    loop.push_back(cones[current]);
  }

  const double closing_gap = (loop.back() - loop.front()).norm();
  if (loop.size() < 6) {
    error = "chain stopped after " + std::to_string(loop.size()) + " of " + std::to_string(n) + " cones";
    return std::nullopt;
  }
  if (closing_gap > MAX_CONE_GAP_M) {
    error = "chain does not close: " + std::to_string(closing_gap) + " m gap after " + std::to_string(loop.size()) + " of " + std::to_string(n) + " cones";
    return std::nullopt;
  }
  if (static_cast<double>(loop.size()) < MIN_CONE_FRACTION * static_cast<double>(n)) {
    error = "only " + std::to_string(loop.size()) + " of " + std::to_string(n) + " cones are on the loop";
    return std::nullopt;
  }
  return loop;
}

/** Builds gates from the blue (left) and yellow (right) loops. Centreline runs with blue on its left. */
std::optional<Gates> build_gates(const std::vector<Vec2>& blue_loop, const std::vector<Vec2>& yellow_loop, std::string& error) {
  const std::vector<Vec2> blue = densify_closed(blue_loop, DENSE_STEP_M);
  const std::vector<Vec2> yellow = densify_closed(yellow_loop, DENSE_STEP_M);

  // Centreline: halfway between each blue point and the closest yellow point, evened out to one point per GATE_SPACING_M
  std::vector<Vec2> rough;
  rough.reserve(blue.size());
  for (const Vec2& b : blue) rough.push_back(0.5 * (b + yellow[nearest(yellow, b)]));
  std::vector<Vec2> centre = resample_closed(rough, GATE_SPACING_M);

  // The centreline must run with blue on its left: if most blue points are on its right, run it the other way
  auto t = tangents(centre);
  int blue_left_votes = 0;
  for (std::size_t i = 0; i < centre.size(); ++i) {
    const Vec2 b = blue[nearest(blue, centre[i])];
    blue_left_votes += (b - centre[i]).dot(left_of(t[i])) > 0.0 ? 1 : -1;
  }
  if (blue_left_votes < 0) {
    std::reverse(centre.begin(), centre.end());
    t = tangents(centre);
  }

  Gates gates;
  gates.centre = centre;
  gates.normal.reserve(centre.size());
  gates.left.reserve(centre.size());
  gates.right.reserve(centre.size());
  for (std::size_t i = 0; i < centre.size(); ++i) {
    // Track width on each side = distance to the nearest point of that cone line. In a corner this is a little less than the
    // distance straight across, which only keeps the line slightly further from the cones
    const double left = (blue[nearest(blue, centre[i])] - centre[i]).norm();
    const double right = (yellow[nearest(yellow, centre[i])] - centre[i]).norm();
    if (left >= MAX_HALF_WIDTH_M || right >= MAX_HALF_WIDTH_M) {
      error = std::string("no ") + (left < MAX_HALF_WIDTH_M ? "yellow" : "blue") + " cone line near gate " + std::to_string(i);
      return std::nullopt;
    }
    gates.normal.push_back(left_of(t[i]));
    gates.left.push_back(left);
    gates.right.push_back(right);
  }
  return gates;
}

/**
 * Sideways offset (m, positive = left) at every gate that minimises curvature, within the cone lines minus the margin.
 * A gate narrower than twice the margin is pinned to the middle of the track.
 */
std::vector<double> solve_offsets(const Gates& gates, double margin) {
  const int N = static_cast<int>(gates.centre.size());

  // Offset limits: stay `margin` inside each cone line. A gate too narrow for that is pinned to the middle of the track.
  Eigen::VectorXd lo(N), hi(N);
  for (int i = 0; i < N; ++i) {
    lo[i] = -(gates.right[i] - margin);
    hi[i] = gates.left[i] - margin;
    if (lo[i] > hi[i]) lo[i] = hi[i] = 0.5 * (gates.left[i] - gates.right[i]);
  }

  // Cost = sum_k |p[k-1] - 2 p[k] + p[k+1]|^2 with p[i] = centre[i] + normal[i] * d[i], i.e. 1/2 d'Pd + q'd.
  // Each term is the path's curvature * spacing^2, so the sum is how much the path bends
  std::vector<Eigen::Triplet<double>> entries;
  entries.reserve(9 * N + N);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(N);
  for (int k = 0; k < N; ++k) {
    const int idx[3] = {(k + N - 1) % N, k, (k + 1) % N};
    const double coef[3] = {1.0, -2.0, 1.0};
    const Vec2 second_difference = gates.centre[idx[0]] - 2.0 * gates.centre[idx[1]] + gates.centre[idx[2]];
    for (int a = 0; a < 3; ++a) {
      q[idx[a]] += 2.0 * coef[a] * gates.normal[idx[a]].dot(second_difference);
      for (int b = 0; b < 3; ++b) {
        entries.emplace_back(idx[a], idx[b], 2.0 * coef[a] * coef[b] * gates.normal[idx[a]].dot(gates.normal[idx[b]]));
      }
    }
  }
  const double alpha = 1.6;  // over-relaxation
  for (int i = 0; i < N; ++i) entries.emplace_back(i, i, 2.0 * REGULARISATION + ADMM_RHO);  // + rho: the ADMM matrix is P + rho I
  Eigen::SparseMatrix<double> K(N, N);
  K.setFromTriplets(entries.begin(), entries.end());

  // ADMM (the algorithm inside OSQP) for the box-constrained QP: the matrix is factored once, each iteration is a back-solve
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> factor(K);
  Eigen::VectorXd z(N);
  for (int i = 0; i < N; ++i) z[i] = std::clamp(0.0, lo[i], hi[i]);
  Eigen::VectorXd u = Eigen::VectorXd::Zero(N);
  for (int it = 0; it < MAX_ITERATIONS; ++it) {
    const Eigen::VectorXd x = factor.solve(ADMM_RHO * (z - u) - q);
    const Eigen::VectorXd x_hat = alpha * x + (1.0 - alpha) * z;
    Eigen::VectorXd z_new(N);
    for (int i = 0; i < N; ++i) z_new[i] = std::clamp(x_hat[i] + u[i], lo[i], hi[i]);
    u += x_hat - z_new;
    const double change = (z_new - z).lpNorm<Eigen::Infinity>();
    const double gap = (x - z_new).lpNorm<Eigen::Infinity>();
    z = z_new;
    if (change < TOLERANCE_M && gap < TOLERANCE_M) break;
  }
  return std::vector<double>(z.data(), z.data() + N);  // z satisfies the limits exactly
}

/** Everything above in one call. The result is a closed line, in driving direction, one point per gate. */
std::optional<std::vector<Vec2>> compute(const std::vector<Vec2>& blue_cones, const std::vector<Vec2>& yellow_cones, double margin,
                                         std::string& error) {
  std::string why;
  const auto blue = order_closed_loop(blue_cones, why);
  if (!blue) {
    error = "blue: " + why;
    return std::nullopt;
  }
  const auto yellow = order_closed_loop(yellow_cones, why);
  if (!yellow) {
    error = "yellow: " + why;
    return std::nullopt;
  }
  const auto gates = build_gates(*blue, *yellow, why);
  if (!gates) {
    error = "gates: " + why;
    return std::nullopt;
  }
  const std::vector<double> d = solve_offsets(*gates, margin);
  std::vector<Vec2> line(gates->centre.size());
  for (std::size_t i = 0; i < line.size(); ++i) line[i] = gates->centre[i] + gates->normal[i] * d[i];
  return line;
}

/**
 * The next WINDOW_POINTS points of a closed line, starting at the point closest to pos among those that run the way the car points.
 * Returns nullopt, so the caller falls back to the midline planner, when no such point is within MAX_WINDOW_DISTANCE_M:
 * the car is pointing against the line, or is far from it. A path that starts far away is never better than no path.
 */
std::optional<std::vector<Vec2>> window_ahead(const std::vector<Vec2>& closed_line, const Vec2& pos, const Vec2& heading) {
  const std::size_t N = closed_line.size();
  if (N < 3) return std::nullopt;
  // Closest point whose direction of travel agrees with the car's heading (rules out the other leg of a hairpin)
  std::size_t best = N;
  double best_d = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < N; ++i) {
    const Vec2 tangent = closed_line[(i + 1) % N] - closed_line[(i + N - 1) % N];
    if (tangent.dot(heading) <= 0.0) continue;
    const double d = (closed_line[i] - pos).squaredNorm();
    if (d < best_d) {
      best_d = d;
      best = i;
    }
  }
  if (best == N || best_d > MAX_WINDOW_DISTANCE_M * MAX_WINDOW_DISTANCE_M) return std::nullopt;
  std::vector<Vec2> out;
  const std::size_t count = std::min(WINDOW_POINTS, N);
  out.reserve(count);
  for (std::size_t k = 0; k < count; ++k) out.push_back(closed_line[(best + k) % N]);
  return out;
}

}  // namespace

bool RacingLinePlanner::init() {
  auto& foxglove = core::FoxgloveServer::instance();
  auto enabled = foxglove.get_param<bool>("racing_line/enabled");
  auto car_half_width = foxglove.get_param<float>("racing_line/car_half_width_m");
  auto cone_radius = foxglove.get_param<float>("racing_line/cone_radius_m");
  auto clearance = foxglove.get_param<float>("racing_line/clearance_m");
  if (!enabled || !car_half_width || !cone_radius || !clearance) {
    spdlog::error("Couldn't load all params for racing line planner.");
    return false;
  }
  _enabled = *enabled;
  _car_half_width = *car_half_width;
  _cone_radius = *cone_radius;
  _clearance = *clearance;

  _param_connection = foxglove.register_param_callback([this](const std::unordered_map<std::string, core::DBParam>& params) {
    if (auto v = core::process_param_update<bool>(params, "racing_line/enabled")) _enabled = *v;
    if (auto v = core::process_param_update<float>(params, "racing_line/car_half_width_m")) _car_half_width = *v;
    if (auto v = core::process_param_update<float>(params, "racing_line/cone_radius_m")) _cone_radius = *v;
    if (auto v = core::process_param_update<float>(params, "racing_line/clearance_m")) _clearance = *v;
  });
  spdlog::info("Racing line planner {}: midline until the track closes, then the racing line", _enabled ? "enabled" : "disabled");
  return true;
}

std::optional<std::vector<core::xyz_vec<float>>> RacingLinePlanner::plan(const dv_msgs::Cones& cones, std::uint64_t scan_ns) {
  const auto period_ns = static_cast<std::uint64_t>(REPLAN_PERIOD_S * 1e9);
  if (!_last_compute_ns || scan_ns < *_last_compute_ns || scan_ns - *_last_compute_ns >= period_ns) {
    std::vector<Vec2> blue, yellow;
    for (const auto& cone : cones.cones()) {
      if (cone.color() == dv_msgs::Cones_ConeColor_BLUE) {
        blue.emplace_back(cone.position().x(), cone.position().y());
      } else if (cone.color() == dv_msgs::Cones_ConeColor_YELLOW) {
        yellow.emplace_back(cone.position().x(), cone.position().y());
      }  // orange and unknown cones are not part of either boundary
    }

    std::string error;
    const double margin = _car_half_width + _cone_radius + _clearance;  // how far the car's centre stays from each cone line
    _line = compute(blue, yellow, margin, error);
    _last_compute_ns = scan_ns;

    if (_last_reported_ok != _line.has_value()) {
      if (_line) {
        spdlog::info("Racing line: track is closed, built a {}-point minimum-curvature line ({} blue, {} yellow cones)", _line->size(),
                     blue.size(), yellow.size());
      } else {
        spdlog::info("Racing line: not available, using the midline planner ({}; map has {} blue, {} yellow cones)", error, blue.size(), yellow.size());
      }
      _last_reported_ok = _line.has_value();
    }
  }
  if (!_line || !_enabled) return std::nullopt;

  // Switch as soon as the track closes: window_ahead picks up the line wherever the car is on it
  const auto vehicle_state = core::StateTracker::instance().vehicle_state().first;
  const Vec2 position(vehicle_state.vehicle_position_map_frame.x, vehicle_state.vehicle_position_map_frame.y);
  const Vec2 heading(vehicle_state.vehicle_heading_map_frame_unit_vector.x, vehicle_state.vehicle_heading_map_frame_unit_vector.y);
  const auto ahead = window_ahead(*_line, position, heading);
  if (!ahead) return std::nullopt;

  std::vector<core::xyz_vec<float>> path;
  path.reserve(ahead->size());
  for (const auto& p : *ahead) path.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), 0.0f});
  return path;
}

}
