#include "RacingLine.hpp"

#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <limits>

namespace planning::racing_line {

namespace {

constexpr double DENSE_STEP_M = 0.25;       // spacing of the finely sampled cone lines used to measure the track
constexpr double DUPLICATE_CONE_M = 0.3;    // cones closer than this are one cone seen twice
constexpr int SMOOTHING_PASSES = 10;        // low-pass passes over the raw centreline

void set_error(std::string* error, std::string message) {
  if (error) *error = std::move(message);
}

double closed_length(const std::vector<Vec2>& p) {
  double length = 0.0;
  for (std::size_t i = 0; i < p.size(); ++i) length += (p[(i + 1) % p.size()] - p[i]).norm();
  return length;
}

/** Point at t in [0,1] between p1 and p2 on the centripetal Catmull-Rom spline through p0..p3 (does not overshoot) */
Vec2 catmull_rom(const Vec2& p0, const Vec2& p1, const Vec2& p2, const Vec2& p3, double t) {
  auto knot = [](const Vec2& a, const Vec2& b) { return std::sqrt(std::max((b - a).norm(), 1e-6)); };
  const double t0 = 0.0;
  const double t1 = t0 + knot(p0, p1);
  const double t2 = t1 + knot(p1, p2);
  const double t3 = t2 + knot(p2, p3);
  const double tt = t1 + t * (t2 - t1);
  const Vec2 a1 = (t1 - tt) / (t1 - t0) * p0 + (tt - t0) / (t1 - t0) * p1;
  const Vec2 a2 = (t2 - tt) / (t2 - t1) * p1 + (tt - t1) / (t2 - t1) * p2;
  const Vec2 a3 = (t3 - tt) / (t3 - t2) * p2 + (tt - t2) / (t3 - t2) * p3;
  const Vec2 b1 = (t2 - tt) / (t2 - t0) * a1 + (tt - t0) / (t2 - t0) * a2;
  const Vec2 b2 = (t3 - tt) / (t3 - t1) * a2 + (tt - t1) / (t3 - t1) * a3;
  return (t2 - tt) / (t2 - t1) * b1 + (tt - t1) / (t2 - t1) * b2;
}

/** A smooth closed curve through the loop, sampled about every `step` metres */
std::vector<Vec2> densify_closed(const std::vector<Vec2>& loop, double step) {
  const std::size_t n = loop.size();
  std::vector<Vec2> out;
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2& p0 = loop[(i + n - 1) % n];
    const Vec2& p1 = loop[i];
    const Vec2& p2 = loop[(i + 1) % n];
    const Vec2& p3 = loop[(i + 2) % n];
    const int k = std::max(1, static_cast<int>(std::ceil((p2 - p1).norm() / step)));
    for (int s = 0; s < k; ++s) out.push_back(catmull_rom(p0, p1, p2, p3, static_cast<double>(s) / k));
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

/** Low-pass filter that does not shrink the curve (Taubin: a smoothing step then a slightly bigger un-shrinking step) */
void smooth_closed(std::vector<Vec2>& path, int passes) {
  const std::size_t n = path.size();
  auto step = [&](double factor) {
    std::vector<Vec2> next(n);
    for (std::size_t i = 0; i < n; ++i) {
      const Vec2 average = 0.5 * (path[(i + n - 1) % n] + path[(i + 1) % n]);
      next[i] = path[i] + factor * (average - path[i]);
    }
    path = std::move(next);
  };
  for (int k = 0; k < passes; ++k) {
    step(0.5);
    step(-0.53);
  }
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

/**
 * Distance from centre c to the cone line straight across the track on one side (sign = +1 left, -1 right).
 * Takes the point of the line with the smallest along-track offset on that side. nullopt if there is none.
 */
std::optional<double> width_to(const std::vector<Vec2>& line, const Vec2& c, const Vec2& tangent, const Vec2& normal, int sign,
                               double max_half_width) {
  double best_lon = std::numeric_limits<double>::max();
  std::optional<double> width;
  for (const Vec2& q : line) {
    const Vec2 rel = q - c;
    const double lat = rel.dot(normal) * sign;
    if (lat <= 0.0 || lat >= max_half_width) continue;
    const double lon = std::abs(rel.dot(tangent));
    if (lon < best_lon) {
      best_lon = lon;
      width = lat;
    }
  }
  return width;
}

}  // namespace

std::optional<std::vector<Vec2>> order_closed_loop(const std::vector<Vec2>& cones, const Params& params, std::string* error) {
  const std::size_t n = cones.size();
  if (n < 6) {
    set_error(error, "fewer than 6 cones");
    return std::nullopt;
  }

  // Walk from cone to cone: always the nearest unvisited cone within the gap limit that is not behind the walk direction
  std::vector<char> used(n, 0);
  std::vector<Vec2> loop{cones[0]};
  used[0] = 1;
  std::size_t current = 0;
  std::size_t consumed = 1;
  Vec2 previous_step = Vec2::Zero();
  bool have_step = false;

  while (true) {
    for (std::size_t j = 0; j < n; ++j) {  // a cone seen twice is one cone
      if (!used[j] && (cones[j] - cones[current]).norm() < DUPLICATE_CONE_M) {
        used[j] = 1;
        ++consumed;
      }
    }
    std::size_t best = n;
    double best_cost = std::numeric_limits<double>::max();
    for (std::size_t j = 0; j < n; ++j) {
      if (used[j]) continue;
      const Vec2 step = cones[j] - cones[current];
      const double d = step.norm();
      if (d > params.max_cone_gap_m) continue;
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
    ++consumed;
    loop.push_back(cones[current]);
  }

  const double closing_gap = (loop.back() - loop.front()).norm();
  if (loop.size() < 6) {
    set_error(error, "chain stopped after " + std::to_string(loop.size()) + " of " + std::to_string(n) + " cones");
    return std::nullopt;
  }
  if (closing_gap > params.max_cone_gap_m) {
    set_error(error, "chain does not close: " + std::to_string(closing_gap) + " m gap after " + std::to_string(loop.size()) + " of " +
                         std::to_string(n) + " cones");
    return std::nullopt;
  }
  if (static_cast<double>(consumed) < params.min_cone_fraction * static_cast<double>(n)) {
    set_error(error, "only " + std::to_string(consumed) + " of " + std::to_string(n) + " cones are on the loop");
    return std::nullopt;
  }
  if (closed_length(loop) < params.min_loop_length_m) {
    set_error(error, "loop is only " + std::to_string(closed_length(loop)) + " m long");
    return std::nullopt;
  }
  return loop;
}

std::optional<Gates> build_gates(const std::vector<Vec2>& blue_loop, const std::vector<Vec2>& yellow_loop, const Params& params,
                                 std::string* error) {
  if (blue_loop.size() < 4 || yellow_loop.size() < 4) {
    set_error(error, "a cone loop has fewer than 4 cones");
    return std::nullopt;
  }
  const std::vector<Vec2> blue = densify_closed(blue_loop, DENSE_STEP_M);
  const std::vector<Vec2> yellow = densify_closed(yellow_loop, DENSE_STEP_M);

  // Rough centreline: halfway between each blue point and the closest yellow point, then evened out and smoothed
  std::vector<Vec2> rough;
  rough.reserve(blue.size());
  for (const Vec2& b : blue) rough.push_back(0.5 * (b + yellow[nearest(yellow, b)]));
  std::vector<Vec2> centre = resample_closed(rough, params.gate_spacing_m);
  smooth_closed(centre, SMOOTHING_PASSES);
  centre = resample_closed(centre, params.gate_spacing_m);

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
    const Vec2 n = left_of(t[i]);
    const auto left = width_to(blue, centre[i], t[i], n, +1, params.max_half_width_m);
    const auto right = width_to(yellow, centre[i], t[i], n, -1, params.max_half_width_m);
    if (!left || !right) {
      set_error(error, std::string("no ") + (left ? "yellow" : "blue") + " cone line across gate " + std::to_string(i));
      return std::nullopt;
    }
    gates.normal.push_back(n);
    gates.left.push_back(*left);
    gates.right.push_back(*right);
  }
  return gates;
}

std::vector<double> solve_offsets(const Gates& gates, const Params& params, int* iterations_used) {
  const int N = static_cast<int>(gates.centre.size());

  // Offset limits: stay `margin` inside each cone line. A gate too narrow for that is pinned to the middle of the track.
  Eigen::VectorXd lo(N), hi(N);
  for (int i = 0; i < N; ++i) {
    lo[i] = -(gates.right[i] - params.margin());
    hi[i] = gates.left[i] - params.margin();
    if (lo[i] > hi[i]) lo[i] = hi[i] = 0.5 * (gates.left[i] - gates.right[i]);
  }

  // Cost = sum_k w[k] |p[k-1] - 2 p[k] + p[k+1]|^2 with p[i] = centre[i] + normal[i] * d[i], i.e. 1/2 d'Pd + q'd.
  //
  // |p[k-1] - 2 p[k] + p[k+1]| equals (curvature * spacing^2) only if the points are evenly spaced along the path. A line that
  // cuts a corner bunches its points up, so unweighted it looks straighter than it is and the solver is rewarded for tightening
  // corners. So each term is weighted w[k] = (nominal spacing / actual local spacing)^4, which makes it curvature^2, and the
  // problem is solved again with spacings measured on the previous answer (iteratively reweighted least squares).
  const double rho = params.admm_rho;
  const double alpha = 1.6;  // over-relaxation

  Eigen::VectorXd z = Eigen::VectorXd::Zero(N);
  for (int i = 0; i < N; ++i) z[i] = std::clamp(0.0, lo[i], hi[i]);
  Eigen::VectorXd u = Eigen::VectorXd::Zero(N);
  Eigen::VectorXd w = Eigen::VectorXd::Ones(N);
  int total_iterations = 0;

  for (int pass = 0; pass <= params.reweight_passes; ++pass) {
    if (pass > 0) {
      std::vector<Vec2> p(N);
      for (int i = 0; i < N; ++i) p[i] = gates.centre[i] + gates.normal[i] * z[i];
      for (int k = 0; k < N; ++k) {
        const double h = 0.5 * ((p[k] - p[(k + N - 1) % N]).norm() + (p[(k + 1) % N] - p[k]).norm());
        const double ratio = std::clamp(params.gate_spacing_m / std::max(h, 1e-6), 0.25, 4.0);
        const double w_new = ratio * ratio * ratio * ratio;
        w[k] = 0.5 * (w[k] + w_new);  // damped, so the reweighting settles instead of oscillating
      }
    }

    std::vector<Eigen::Triplet<double>> entries;
    entries.reserve(9 * N + N);
    Eigen::VectorXd q = Eigen::VectorXd::Zero(N);
    for (int k = 0; k < N; ++k) {
      const int idx[3] = {(k + N - 1) % N, k, (k + 1) % N};
      const double coef[3] = {1.0, -2.0, 1.0};
      const Vec2 second_difference = gates.centre[idx[0]] - 2.0 * gates.centre[idx[1]] + gates.centre[idx[2]];
      for (int a = 0; a < 3; ++a) {
        q[idx[a]] += 2.0 * w[k] * coef[a] * gates.normal[idx[a]].dot(second_difference);
        for (int b = 0; b < 3; ++b) {
          entries.emplace_back(idx[a], idx[b], 2.0 * w[k] * coef[a] * coef[b] * gates.normal[idx[a]].dot(gates.normal[idx[b]]));
        }
      }
    }
    for (int i = 0; i < N; ++i) entries.emplace_back(i, i, 2.0 * params.regularisation + rho);  // + rho: the ADMM matrix is P + rho I
    Eigen::SparseMatrix<double> K(N, N);
    K.setFromTriplets(entries.begin(), entries.end());

    // ADMM (the algorithm inside OSQP) for the box-constrained QP: the matrix is factored once per pass, each iteration is a back-solve
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> factor(K);
    Eigen::VectorXd x = z;
    for (int it = 0; it < params.max_iterations; ++it) {
      ++total_iterations;
      x = factor.solve(rho * (z - u) - q);
      const Eigen::VectorXd x_hat = alpha * x + (1.0 - alpha) * z;
      Eigen::VectorXd z_new(N);
      for (int i = 0; i < N; ++i) z_new[i] = std::clamp(x_hat[i] + u[i], lo[i], hi[i]);
      u += x_hat - z_new;
      const double change = (z_new - z).lpNorm<Eigen::Infinity>();
      const double gap = (x - z_new).lpNorm<Eigen::Infinity>();
      z = z_new;
      if (change < params.tolerance_m && gap < params.tolerance_m) break;
    }
  }
  if (iterations_used) *iterations_used = total_iterations;
  return std::vector<double>(z.data(), z.data() + N);  // z satisfies the limits exactly
}

std::optional<std::vector<Vec2>> compute(const std::vector<Vec2>& blue_cones, const std::vector<Vec2>& yellow_cones, const Params& params,
                                         std::string* error) {
  std::string why;
  const auto blue = order_closed_loop(blue_cones, params, &why);
  if (!blue) {
    set_error(error, "blue: " + why);
    return std::nullopt;
  }
  const auto yellow = order_closed_loop(yellow_cones, params, &why);
  if (!yellow) {
    set_error(error, "yellow: " + why);
    return std::nullopt;
  }
  const auto gates = build_gates(*blue, *yellow, params, &why);
  if (!gates) {
    set_error(error, "gates: " + why);
    return std::nullopt;
  }
  const std::vector<double> d = solve_offsets(*gates, params);
  std::vector<Vec2> line(gates->centre.size());
  for (std::size_t i = 0; i < line.size(); ++i) line[i] = gates->centre[i] + gates->normal[i] * d[i];
  return line;
}

std::optional<std::vector<Vec2>> window_ahead(const std::vector<Vec2>& closed_line, const Vec2& pos, const Vec2& heading, std::size_t n_points,
                                              double max_distance_m) {
  const std::size_t N = closed_line.size();
  if (N < 3 || n_points == 0) return std::nullopt;
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
  if (best == N || best_d > max_distance_m * max_distance_m) return std::nullopt;
  std::vector<Vec2> out;
  const std::size_t count = std::min(n_points, N);
  out.reserve(count);
  for (std::size_t k = 0; k < count; ++k) out.push_back(closed_line[(best + k) % N]);
  return out;
}

std::vector<double> curvature(const std::vector<Vec2>& p) {
  const std::size_t n = p.size();
  std::vector<double> k(n, 0.0);
  for (std::size_t i = 0; i < n; ++i) {
    const Vec2& a = p[(i + n - 1) % n];
    const Vec2& b = p[i];
    const Vec2& c = p[(i + 1) % n];
    const Vec2 ab = b - a, bc = c - b, ca = a - c;
    const double cross = ab.x() * bc.y() - ab.y() * bc.x();
    const double den = ab.norm() * bc.norm() * ca.norm();
    k[i] = den > 1e-9 ? 2.0 * cross / den : 0.0;
  }
  return k;
}

}  // namespace planning::racing_line
