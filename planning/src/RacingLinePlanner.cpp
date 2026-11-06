#include "RacingLinePlanner.hpp"

#include <string>

#include <algorithm>
#include <spdlog/spdlog.h>

namespace planning {

RacingLinePlanner::RacingLinePlanner() = default;

RacingLinePlanner::RacingLinePlanner(Config config) : _config(std::move(config)) {}

std::optional<std::vector<core::xyz_vec<float>>> RacingLinePlanner::plan(const dv_msgs::Cones& cones, std::uint64_t scan_ns) {
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
        spdlog::info("Racing line: track is closed, built a {}-point minimum-curvature line ({} blue, {} yellow cones)", _line->size(),
                     blue.size(), yellow.size());
      } else {
        spdlog::info("Racing line: not available, using the midline planner ({}; map has {} blue, {} yellow cones)", error, blue.size(), yellow.size());
      }
      _last_reported_ok = _line.has_value();
    }
  }
  if (_line) _ever_closed = true;

  const auto vehicle_state = core::StateTracker::instance().vehicle_state().first;
  const racing_line::Vec2 position(vehicle_state.vehicle_position_map_frame.x, vehicle_state.vehicle_position_map_frame.y);
  const racing_line::Vec2 heading(vehicle_state.vehicle_heading_map_frame_unit_vector.x, vehicle_state.vehicle_heading_map_frame_unit_vector.y);

  // Count laps (also while the line is not available yet: lap 1 is when the track is discovered)
  constexpr double LEAVE_START_M = 15.0;  // must first get this far from the start
  constexpr double LAP_DONE_M = 4.0;      // back within this of the start = lap done
  if (!_start_position) {
    _start_position = position;
  } else {
    const double from_start = (position - *_start_position).norm();
    if (from_start > LEAVE_START_M) _left_start = true;
    if (_left_start && from_start < LAP_DONE_M) {
      _left_start = false;
      ++_laps_completed;
      spdlog::info("Racing line: lap {} complete", _laps_completed);
    }
  }
  const int discovery_laps = 1;
  const int last_lap = discovery_laps + std::max(_config.midline_laps_after_closure, 0) + std::max(_config.racing_laps, 1);
  if (_config.enabled && _laps_completed >= last_lap && !_finished) {
    _finished = true;
    spdlog::info("Racing line: all {} laps complete (1 discovery, {} midline, {} racing line)", last_lap, _config.midline_laps_after_closure,
                 _config.racing_laps);
  }

  if (!_line || !_config.enabled) return std::nullopt;

  // Hold the midline until the discovery lap and the midline laps are done, so the racing line starts on the same closed map
  if (!_racing_started) {
    if (_laps_completed < discovery_laps + std::max(_config.midline_laps_after_closure, 0)) return std::nullopt;
    _racing_started = true;
    spdlog::info("Racing line: {} lap(s) complete, switching to the racing line", _laps_completed);
  }

  const auto ahead = racing_line::window_ahead(*_line, position, heading, _config.window_points);
  if (!ahead) return std::nullopt;

  std::vector<core::xyz_vec<float>> path;
  path.reserve(ahead->size());
  for (const auto& p : *ahead) path.push_back({static_cast<float>(p.x()), static_cast<float>(p.y()), 0.0f});
  return path;
}

}  // namespace planning
