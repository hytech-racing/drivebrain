#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

namespace control {

/// No defaults on purpose: the values come from the "speed_profile" block of drivebrain_config.json.
struct SpeedProfileParams {
    float max_speed_mps;       // top speed on straights
    float min_speed_mps;       // never slower than this, so the car keeps moving through the tightest corner
    float max_lat_accel_mps2;  // sideways acceleration allowed in a corner: v <= sqrt(max_lat_accel / curvature)
    float max_brake_mps2;      // how hard the car may slow down for a corner ahead
    float max_accel_mps2;      // how fast the commanded speed may rise (the car's torque limit alone is not a gentle limit)
};

/**
 * Target speed for every point of a path (any point type with x and y), in the order the car reaches them.
 *
 * Each point gets the fastest speed the corner there allows, sqrt(max_lat_accel / curvature), capped to
 * [min_speed, max_speed]. A backward pass then lowers the speeds before a corner so the car can brake down to it
 * at max_brake. Speeding up is limited separately, by the controller, which raises its commanded speed by at most
 * max_accel per second.
 * The end of the path is only the end of the window the planner handed over, so it does not force the car to stop.
 */
template <class Point>
std::vector<float> speed_profile(const std::vector<Point>& path, const SpeedProfileParams& p) {
    const std::size_t n = path.size();
    if (n < 2) return std::vector<float>(n, p.max_speed_mps);

    auto dist = [&](std::size_t a, std::size_t b) { return std::hypot(double(path[b].x) - path[a].x, double(path[b].y) - path[a].y); };

    // Curvature at each interior point from the circle through it and its neighbours (Menger curvature)
    std::vector<double> kappa(n, 0.0);
    for (std::size_t i = 1; i + 1 < n; ++i) {
        const double a = dist(i - 1, i), b = dist(i, i + 1), c = dist(i - 1, i + 1);
        if (a < 1e-3 || b < 1e-3 || c < 1e-3) continue;
        const double cross = (double(path[i].x) - path[i - 1].x) * (double(path[i + 1].y) - path[i].y) -
                             (double(path[i].y) - path[i - 1].y) * (double(path[i + 1].x) - path[i].x);
        kappa[i] = 2.0 * std::fabs(cross) / (a * b * c);
    }
    kappa[0] = kappa[1];
    kappa[n - 1] = kappa[n - 2];

    // A little smoothing, so one jittery point in the path does not become a speed spike
    std::vector<double> smooth(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double l = kappa[i > 0 ? i - 1 : i], r = kappa[i + 1 < n ? i + 1 : i];
        smooth[i] = 0.25 * l + 0.5 * kappa[i] + 0.25 * r;
    }

    std::vector<float> v(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double limit = std::sqrt(p.max_lat_accel_mps2 / std::max(smooth[i], 1e-3));
        v[i] = static_cast<float>(std::clamp(limit, double(p.min_speed_mps), double(p.max_speed_mps)));
    }

    for (std::size_t i = n - 1; i-- > 0;) {
        const double reachable = std::sqrt(double(v[i + 1]) * v[i + 1] + 2.0 * p.max_brake_mps2 * dist(i, i + 1));
        v[i] = static_cast<float>(std::min<double>(v[i], reachable));
    }
    return v;
}

}  // namespace control
