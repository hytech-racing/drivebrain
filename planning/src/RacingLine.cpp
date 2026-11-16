#include "RacingLine.hpp"

#include <FoxgloveServer.hpp>

#include <Eigen/Sparse>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cmath>
#include <chrono>
#include <limits>
#include <string>

namespace planning {

namespace {

using Vec2 = Eigen::Vector2d;

constexpr double GATE_SPACING_M = 1.0;
constexpr double MAX_CONE_GAP_M = 11.0;        // cones are up to 5 m apart, so this survives one missing cone
constexpr double MIN_CONE_FRACTION = 0.85;     // share of a colour's cones that must lie on its loop (rest are strays)
constexpr double MAX_HALF_WIDTH_M = 4.0;
constexpr double FILL_STEP_M = 0.25;
constexpr double REGULARISATION = 1e-4;        // tiny pull towards the centreline where curvature does not care
constexpr int MAX_ITERATIONS = 5000;
constexpr double ADMM_RHO = 0.03;              // only changes how fast the solver converges, not the answer
constexpr double ALPHA = 1.6;                  // over-relaxation, also only changes how fast it converges
constexpr double TOLERANCE_M = 5e-4;
constexpr std::size_t POINTS_AHEAD = 30;
constexpr double MAX_DISTANCE_TO_LINE_M = 5.0;

/** normal[i] points left; left[i]/right[i] are distances from centre[i] to the cone lines */
struct Gates {
  std::vector<Vec2> centre;
  std::vector<Vec2> normal;
  std::vector<double> left;
  std::vector<double> right;
};

/** Points every `step` metres along the straight lines between a loop of cones, including from the last cone back to the first */
std::vector<Vec2> fill_between_cones(const std::vector<Vec2>& loop, double step) {
  std::vector<Vec2> out;
  for (std::size_t i = 0; i < loop.size(); ++i) {
    const Vec2& start = loop[i];
    const Vec2& end = loop[(i + 1) % loop.size()];
    const int pieces = std::max(1, static_cast<int>(std::ceil((end - start).norm() / step)));
    for (int p = 0; p < pieces; ++p) out.push_back(start + (end - start) * (static_cast<double>(p) / pieces));
  }
  return out;
}

/** New points about `spacing` metres apart along a loop, replacing its unevenly spaced ones */
std::vector<Vec2> evenly_space(const std::vector<Vec2>& path, double spacing) {
  const std::size_t n = path.size();
  std::vector<double> dist_along(n + 1, 0.0);
  for (std::size_t i = 0; i < n; ++i) dist_along[i + 1] = dist_along[i] + (path[(i + 1) % n] - path[i]).norm();
  const double total_length = dist_along[n];
  const std::size_t count = std::max<std::size_t>(8, static_cast<std::size_t>(std::lround(total_length / spacing)));
  std::vector<Vec2> out;
  out.reserve(count);
  std::size_t segment = 0;
  for (std::size_t k = 0; k < count; ++k) {
    const double target = total_length * static_cast<double>(k) / static_cast<double>(count);
    while (segment + 1 < n && dist_along[segment + 1] < target) ++segment;
    const double segment_length = dist_along[segment + 1] - dist_along[segment];
    const double fraction = segment_length > 1e-9 ? (target - dist_along[segment]) / segment_length : 0.0;
    out.push_back(path[segment] + fraction * (path[(segment + 1) % n] - path[segment]));
  }
  return out;
}

/** Unit direction of travel at each point of a closed path */
std::vector<Vec2> travel_directions(const std::vector<Vec2>& points) {
  const std::size_t n = points.size();
  std::vector<Vec2> out(n);
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 prev_to_next = points[(i + 1) % n] - points[(i + n - 1) % n];
    out[i] = prev_to_next.norm() > 1e-9 ? Vec2(prev_to_next.normalized()) : Vec2(1.0, 0.0);
  }
  return out;
}

Vec2 left_of(const Vec2& direction) { return {-direction.y(), direction.x()}; }

/** Index of the point closest to `query` */
std::size_t nearest_index(const std::vector<Vec2>& points, const Vec2& query) {
  std::size_t best = 0;
  double best_dist_sq = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < points.size(); ++i) {
    const double dist_sq = (points[i] - query).squaredNorm();
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best = i;
    }
  }
  return best;
}

/**
 * Orders one colour's cones into a closed loop, or nullopt with the reason (open track, gap too big, too many strays).
 * Walks from cone to cone, always to the nearest unvisited cone within MAX_CONE_GAP_M, never doubling back more than
 * 90 degrees and preferring straight on over a sharp turn.
 */
std::optional<std::vector<Vec2>> order_into_loop(const std::vector<Vec2>& cones, std::string& error) {
  const std::size_t n = cones.size();
  if (n < 6) {
    error = "fewer than 6 cones";
    return std::nullopt;
  }

  std::vector<char> visited(n, 0);
  std::vector<Vec2> loop{cones[0]};
  visited[0] = 1;
  std::size_t current = 0;
  Vec2 previous_step = Vec2::Zero();
  bool have_step = false;

  while (true) {
    std::size_t best = n;
    double best_cost = std::numeric_limits<double>::max();
    for (std::size_t j = 0; j < n; ++j) {
      if (visited[j]) continue;
      const Vec2 step = cones[j] - cones[current];
      const double dist = step.norm();
      if (dist > MAX_CONE_GAP_M) continue;
      double cost = dist;
      if (have_step) {
        const double cos_turn = step.dot(previous_step) / (dist * previous_step.norm());
        if (cos_turn <= 0.0) continue;
        cost = dist * (2.0 - cos_turn);
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
    visited[current] = 1;
    loop.push_back(cones[current]);
  }

  const double closing_gap = (loop.back() - loop.front()).norm();
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

/**
 * Centreline between the blue (left) and yellow (right) loops, with the track width on each side at every gate.
 * The centreline is turned around if needed so blue is on its left in the driving direction.
 */
std::optional<Gates> build_gates(const std::vector<Vec2>& blue_loop, const std::vector<Vec2>& yellow_loop, std::string& error) {
  const std::vector<Vec2> blue_line = fill_between_cones(blue_loop, FILL_STEP_M);
  const std::vector<Vec2> yellow_line = fill_between_cones(yellow_loop, FILL_STEP_M);

  std::vector<Vec2> rough_centre;
  rough_centre.reserve(blue_line.size());
  for (const Vec2& b : blue_line) rough_centre.push_back(0.5 * (b + yellow_line[nearest_index(yellow_line, b)]));
  std::vector<Vec2> centre = evenly_space(rough_centre, GATE_SPACING_M);

  auto direction = travel_directions(centre);
  int blue_left_votes = 0;
  for (std::size_t i = 0; i < centre.size(); ++i) {
    const Vec2 closest_blue = blue_line[nearest_index(blue_line, centre[i])];
    blue_left_votes += (closest_blue - centre[i]).dot(left_of(direction[i])) > 0.0 ? 1 : -1;
  }
  if (blue_left_votes < 0) {
    std::reverse(centre.begin(), centre.end());
    direction = travel_directions(centre);
  }

  Gates gates;
  gates.centre = centre;
  gates.normal.reserve(centre.size());
  gates.left.reserve(centre.size());
  gates.right.reserve(centre.size());
  for (std::size_t i = 0; i < centre.size(); ++i) {
    const double left = (blue_line[nearest_index(blue_line, centre[i])] - centre[i]).norm();
    const double right = (yellow_line[nearest_index(yellow_line, centre[i])] - centre[i]).norm();
    if (left >= MAX_HALF_WIDTH_M || right >= MAX_HALF_WIDTH_M) {
      error = std::string("no ") + (left < MAX_HALF_WIDTH_M ? "yellow" : "blue") + " cone line near gate " + std::to_string(i);
      return std::nullopt;
    }
    gates.normal.push_back(left_of(direction[i]));
    gates.left.push_back(left);
    gates.right.push_back(right);
  }
  return gates;
}

/**
 * Sideways offset (m, positive = left) at every gate that minimises curvature, staying `margin` inside the cone lines.
 * A gate too narrow for the margin is pinned to the middle of the track.
 *
 * Cost = sum_k |p[k-1] - 2 p[k] + p[k+1]|^2 with p[i] = centre[i] + normal[i] * offset[i], i.e. 1/2 offset'P offset + q'offset.
 * Each term is the path's curvature * spacing^2, so the sum is how much the path bends. Solved with ADMM (the algorithm inside
 * OSQP) on P + rho I: solve ignoring the limits, clamp to the limits, carry what clamping removed into the next solve.
 */
std::vector<double> solve_offsets(const Gates& gates, double margin) {
  const int n = static_cast<int>(gates.centre.size());

  Eigen::VectorXd min_offset(n), max_offset(n);
  for (int i = 0; i < n; ++i) {
    min_offset[i] = -(gates.right[i] - margin);
    max_offset[i] = gates.left[i] - margin;
    if (min_offset[i] > max_offset[i]) min_offset[i] = max_offset[i] = 0.5 * (gates.left[i] - gates.right[i]);
  }

  std::vector<Eigen::Triplet<double>> p_entries;
  p_entries.reserve(9 * n + n);
  Eigen::VectorXd q = Eigen::VectorXd::Zero(n);
  for (int k = 0; k < n; ++k) {
    const int points[3] = {(k + n - 1) % n, k, (k + 1) % n};
    const double weights[3] = {1.0, -2.0, 1.0};
    const Vec2 centre_bend = gates.centre[points[0]] - 2.0 * gates.centre[points[1]] + gates.centre[points[2]];
    for (int a = 0; a < 3; ++a) {
      q[points[a]] += 2.0 * weights[a] * gates.normal[points[a]].dot(centre_bend);
      for (int b = 0; b < 3; ++b) {
        p_entries.emplace_back(points[a], points[b], 2.0 * weights[a] * weights[b] * gates.normal[points[a]].dot(gates.normal[points[b]]));
      }
    }
  }
  for (int i = 0; i < n; ++i) p_entries.emplace_back(i, i, 2.0 * REGULARISATION + ADMM_RHO);
  Eigen::SparseMatrix<double> system(n, n);
  system.setFromTriplets(p_entries.begin(), p_entries.end());

  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> factored(system);
  Eigen::VectorXd offsets(n);
  for (int i = 0; i < n; ++i) offsets[i] = std::clamp(0.0, min_offset[i], max_offset[i]);
  Eigen::VectorXd correction = Eigen::VectorXd::Zero(n);
  for (int iteration = 0; iteration < MAX_ITERATIONS; ++iteration) {
    const Eigen::VectorXd free_offsets = factored.solve(ADMM_RHO * (offsets - correction) - q);
    const Eigen::VectorXd relaxed = ALPHA * free_offsets + (1.0 - ALPHA) * offsets;
    Eigen::VectorXd new_offsets(n);
    for (int i = 0; i < n; ++i) new_offsets[i] = std::clamp(relaxed[i] + correction[i], min_offset[i], max_offset[i]);
    correction += relaxed - new_offsets;
    const double change = (new_offsets - offsets).lpNorm<Eigen::Infinity>();
    const double disagreement = (free_offsets - new_offsets).lpNorm<Eigen::Infinity>();
    offsets = new_offsets;
    if (change < TOLERANCE_M && disagreement < TOLERANCE_M) break;
  }
  return std::vector<double>(offsets.data(), offsets.data() + n);
}

/** The whole pipeline: closed racing line in driving direction, one point per gate, or nullopt with the reason */
std::optional<std::vector<Vec2>> build_racing_line(const std::vector<Vec2>& blue_cones, const std::vector<Vec2>& yellow_cones, double margin,
                                                   std::string& error) {
  std::string why;
  const auto blue = order_into_loop(blue_cones, why);
  if (!blue) {
    error = "blue: " + why;
    return std::nullopt;
  }
  const auto yellow = order_into_loop(yellow_cones, why);
  if (!yellow) {
    error = "yellow: " + why;
    return std::nullopt;
  }
  const auto gates = build_gates(*blue, *yellow, why);
  if (!gates) {
    error = "gates: " + why;
    return std::nullopt;
  }
  const std::vector<double> offsets = solve_offsets(*gates, margin);
  std::vector<Vec2> line(gates->centre.size());
  for (std::size_t i = 0; i < line.size(); ++i) line[i] = gates->centre[i] + gates->normal[i] * offsets[i];
  return line;
}

/**
 * The next POINTS_AHEAD points of the line from the car, or nullopt if the car is pointing against it or far from it.
 * Only points running the way the car points are considered, so the other leg of a hairpin is never picked.
 */
std::optional<std::vector<Vec2>> line_ahead_of_car(const std::vector<Vec2>& line, const Vec2& car_position, const Vec2& car_heading) {
  const std::size_t n = line.size();
  if (n < 3) return std::nullopt;
  std::size_t best = n;
  double best_dist_sq = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2 direction = line[(i + 1) % n] - line[(i + n - 1) % n];
    if (direction.dot(car_heading) <= 0.0) continue;
    const double dist_sq = (line[i] - car_position).squaredNorm();
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      best = i;
    }
  }
  if (best == n || best_dist_sq > MAX_DISTANCE_TO_LINE_M * MAX_DISTANCE_TO_LINE_M) return std::nullopt;
  std::vector<Vec2> out;
  const std::size_t count = std::min(POINTS_AHEAD, n);
  out.reserve(count);
  for (std::size_t k = 0; k < count; ++k) out.push_back(line[(best + k) % n]);
  return out;
}

}  // namespace

RacingLinePlanner::~RacingLinePlanner() {
  {
    std::lock_guard lock(_stop_mutex);
    _running = false;
  }
  _stop_signal.notify_one();
  if (_thread.joinable()) _thread.join();
}

bool RacingLinePlanner::init() {
  auto& foxglove = core::FoxgloveServer::instance();
  auto enabled = foxglove.get_param<bool>("racing_line/enabled");
  auto car_half_width = foxglove.get_param<float>("racing_line/car_half_width_m");
  auto cone_radius = foxglove.get_param<float>("racing_line/cone_radius_m");
  auto clearance = foxglove.get_param<float>("racing_line/clearance_m");
  auto replan_hz = foxglove.get_param<float>("racing_line/replan_hz");
  if (!enabled || !car_half_width || !cone_radius || !clearance || !replan_hz) {
    spdlog::error("Couldn't load all params for racing line planner.");
    return false;
  }
  _enabled = *enabled;
  _car_half_width = *car_half_width;
  _cone_radius = *cone_radius;
  _clearance = *clearance;
  _replan_hz = *replan_hz;

  _param_connection = foxglove.register_param_callback([this](const std::unordered_map<std::string, core::DBParam>& params) {
    if (auto v = core::process_param_update<bool>(params, "racing_line/enabled")) _enabled = *v;
    if (auto v = core::process_param_update<float>(params, "racing_line/car_half_width_m")) _car_half_width = *v;
    if (auto v = core::process_param_update<float>(params, "racing_line/cone_radius_m")) _cone_radius = *v;
    if (auto v = core::process_param_update<float>(params, "racing_line/clearance_m")) _clearance = *v;
    if (auto v = core::process_param_update<float>(params, "racing_line/replan_hz")) _replan_hz = *v;
  });

  _running = true;
  _thread = std::thread(&RacingLinePlanner::_run, this);
  spdlog::info("Racing line planner {}: midline until the track closes, then the racing line", _enabled ? "enabled" : "disabled");
  return true;
}

/**
 * Rebuilds the line from the latest SLAM map every 1 / replan_hz seconds until the destructor stops it.
 * replan_hz is floored at 0.1, so a bad value from Foxglove still rebuilds every 10 s. Logs only when availability changes.
 */
void RacingLinePlanner::_run() {
  std::optional<bool> last_reported_ok;
  while (true) {
    const auto start = std::chrono::steady_clock::now();

    if (const auto map = core::StateTracker::instance().dv_state().cone_observations) {
      std::vector<Vec2> blue, yellow;
      for (const auto& cone : map->cones()) {
        if (cone.color() == dv_msgs::Cones_ConeColor_BLUE) {
          blue.emplace_back(cone.position().x(), cone.position().y());
        } else if (cone.color() == dv_msgs::Cones_ConeColor_YELLOW) {
          yellow.emplace_back(cone.position().x(), cone.position().y());
        }
      }

      std::string error;
      const double margin = _car_half_width + _cone_radius + _clearance;
      const auto line = build_racing_line(blue, yellow, margin, error);
      {
        std::lock_guard lock(_line_mutex);
        _line = line ? std::make_shared<const Line>(*line) : nullptr;
      }

      if (last_reported_ok != line.has_value()) {
        if (line) {
          spdlog::info("Racing line: track is closed, built a {}-point minimum-curvature line ({} blue, {} yellow cones)", line->size(),
                       blue.size(), yellow.size());
        } else {
          spdlog::info("Racing line: not available, using the midline planner ({}; map has {} blue, {} yellow cones)", error, blue.size(), yellow.size());
        }
        last_reported_ok = line.has_value();
      }
    }

    const float hz = std::max(_replan_hz.load(), 0.1f);
    std::unique_lock lock(_stop_mutex);
    if (_stop_signal.wait_until(lock, start + std::chrono::duration<float>(1.0f / hz), [this] { return !_running; })) return;
  }
}

/** The latest line from the rebuild thread, nullptr while the map is not closed */
std::shared_ptr<const RacingLinePlanner::Line> RacingLinePlanner::_latest_line() const {
  std::lock_guard lock(_line_mutex);
  return _line;
}

std::optional<std::vector<core::xyz_vec<float>>> RacingLinePlanner::plan() const {
  const auto line = _latest_line();
  if (!line || !_enabled) return std::nullopt;

  const auto vehicle_state = core::StateTracker::instance().vehicle_state().first;
  const Vec2 position(vehicle_state.vehicle_position_map_frame.x, vehicle_state.vehicle_position_map_frame.y);
  const Vec2 heading(vehicle_state.vehicle_heading_map_frame_unit_vector.x, vehicle_state.vehicle_heading_map_frame_unit_vector.y);
  const auto ahead = line_ahead_of_car(*line, position, heading);
  if (!ahead) return std::nullopt;

  std::vector<core::xyz_vec<float>> path;
  path.reserve(ahead->size());
  for (const auto& p : *ahead) path.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), 0.0f});
  return path;
}

}  // namespace planning
