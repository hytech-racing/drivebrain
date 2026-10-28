#include "RacingLinePlanner.hpp"

#include <string>

#include <spdlog/spdlog.h>

namespace planning {

RacingLinePlanner::RacingLinePlanner() = default;

RacingLinePlanner::RacingLinePlanner(Config config) : _config(std::move(config)) {}

std::optional<std::vector<core::xyz_vec<float>>> RacingLinePlanner::plan(const dv_msgs::Cones& cones, std::uint64_t scan_ns) {
  if (!_config.enabled) return std::nullopt;

  const auto period_ns = static_cast<std::uint64_t>(_config.replan_period_s * 1e9);
  if (!_has_computed || scan_ns < _last_compute_ns || scan_ns - _last_compute_ns >= period_ns) {
    std::vector<racing_line::Vec2> blue, yellow;
    for (const auto& cone : cones.cones()) {
      if (cone.color() == dv_msgs::Cones_ConeColor_BLUE) {
        blue.emplace_back(cone.position().x(), cone.position().y());
      } else if (cone.color() == dv_msgs::Cones_ConeColor_YELLOW) {
        yellow.emplace_back(cone.position().x(), cone.position().y());
      }  // orange and unknown cones are not part of either boundary
    }

    std::string error;
    _line = racing_line::compute(blue, yellow, _config.line, &error);
    _has_computed = true;
    _last_compute_ns = scan_ns;

    // Report changes, and repeat the reason every 10 s while unavailable: on lap 1 it fails every second, and that is expected
    constexpr std::uint64_t REPEAT_NS = 10'000'000'000ULL;
    const bool repeat_unavailable = !_line && scan_ns >= _last_report_ns && scan_ns - _last_report_ns >= REPEAT_NS;
    if (_last_reported_ok != _line.has_value() || repeat_unavailable) {
      _last_report_ns = scan_ns;
      if (_line) {
        spdlog::info("Racing line: track is closed, following a {}-point minimum-curvature line ({} blue, {} yellow cones)", _line->size(),
                     blue.size(), yellow.size());
      } else {
        spdlog::info("Racing line: not available, using the midline planner ({}; map has {} blue, {} yellow cones)", error, blue.size(), yellow.size());
      }
      _last_reported_ok = _line.has_value();
    }
  }
  if (!_line) return std::nullopt;

  const auto vehicle_state = core::StateTracker::instance().vehicle_state().first;
  const racing_line::Vec2 position(vehicle_state.vehicle_position_map_frame.x, vehicle_state.vehicle_position_map_frame.y);
  const racing_line::Vec2 heading(vehicle_state.vehicle_heading_map_frame_unit_vector.x, vehicle_state.vehicle_heading_map_frame_unit_vector.y);
  const auto ahead = racing_line::window_ahead(*_line, position, heading, _config.window_points);
  if (!ahead) return std::nullopt;

  std::vector<core::xyz_vec<float>> path;
  path.reserve(ahead->size());
  for (const auto& p : *ahead) path.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), 0.0f});
  return path;
}

}  // namespace planning
