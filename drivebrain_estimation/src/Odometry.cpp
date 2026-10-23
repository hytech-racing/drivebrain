#include "Odometry.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include <FoxgloveServer.hpp>
#include <StateTracker.hpp>
#include <Telemetry.hpp>

using namespace estimation;

namespace {

/* Pose history kept for lookup: one lidar scan (100 ms) plus its processing delay, with margin */
constexpr std::uint64_t HISTORY_NS = 1'000'000'000;
constexpr std::uint64_t PUBLISH_PERIOD_NS = 20'000'000;
constexpr std::uint64_t TRAIL_RENDER_PERIOD_NS = 500'000'000;

}  // namespace

Odometry::Odometry() {
    _position_base_m = {core::FoxgloveServer::instance().get_param<float>("Odometry/oms_position_base_x_m").value(),
                        core::FoxgloveServer::instance().get_param<float>("Odometry/oms_position_base_y_m").value()};
}

bool Odometry::update(const hytech_msgs::OMSData& sample, const std::uint64_t receive_ns) {
    {
        std::scoped_lock lock(_mutex);

        const std::uint64_t ts = _host_time_ns(sample.sample_time_s(), receive_ns);
        if (ts <= _timestamp_ns) {
            return false;
        }

        // TODO: sensoric is configured to face properly now so we should invert this after merge
        Eigen::Vector3d twist(-sample.motion().vel_x_kmh() / 3.6, -sample.motion().vel_y_kmh() / 3.6,
                              sample.rate_hor_deg_s().z() * M_PI / 180.0);
        // velocity at the sensor -> base_link: v_base = v_sensor - w x r
        twist.x() += twist.z() * _position_base_m.y();
        twist.y() -= twist.z() * _position_base_m.x();
        if (sample.status().standstill()) twist.setZero();
        else if (!sample.status().status_ok()) twist = _twist;

        if (_timestamp_ns != 0) {
            _pose = _pose * core::geometry::integrate_twist(0.5 * (_twist + twist), (ts - _timestamp_ns) * 1e-9);
        }
        _timestamp_ns = ts;
        _twist = twist;
        _buffer.emplace_back(ts, _pose);

        // Always keeps the newest sample
        while (_buffer.size() > 1 &&
               _buffer.back().first - _buffer.front().first > HISTORY_NS) {
            _buffer.pop_front();
        }
    }

    _cv.notify_all();
    _publish(_buffer.back().first, _buffer.back().second, _twist);
    return true;
}

std::optional<Eigen::Isometry2d> Odometry::pose(const std::uint64_t timestamp_ns,
                                                const std::chrono::nanoseconds timeout) const {
    std::unique_lock lock(_mutex);

    if (timeout > std::chrono::nanoseconds{0}) {
        // A timestamp older than the buffer will never become available
        _cv.wait_for(lock, timeout, [&] {
            return !_buffer.empty() && (timestamp_ns <= _buffer.back().first ||
                                        timestamp_ns < _buffer.front().first);
        });
    }

    if (_buffer.empty() || timestamp_ns < _buffer.front().first ||
        timestamp_ns > _buffer.back().first) {
        return std::nullopt;
    }

    const auto after = std::lower_bound(
        _buffer.begin(), _buffer.end(), timestamp_ns,
        [](const auto& sample, std::uint64_t stamp_ns) { return sample.first < stamp_ns; });
    if (after->first == timestamp_ns) {
        return after->second;
    }

    const auto before = std::prev(after);
    const double alpha = static_cast<double>(timestamp_ns - before->first) /
                         static_cast<double>(after->first - before->first);

    // Shortest-arc yaw interpolation
    const double yaw_before = core::geometry::yaw(before->second);
    const double yaw_delta =
        std::remainder(core::geometry::yaw(after->second) - yaw_before, 2.0 * M_PI);
    const Eigen::Vector2d position =
        before->second.translation() +
        alpha * (after->second.translation() - before->second.translation());

    return core::geometry::make_pose2d(position.x(), position.y(), yaw_before + alpha * yaw_delta);
}

Eigen::Vector3d Odometry::twist() const {
    std::scoped_lock lock(_mutex);
    return _twist;
}

std::uint64_t Odometry::_host_time_ns(const float sample_time_s, const std::uint64_t receive_ns) {
    const std::int64_t sample_ns = static_cast<std::int64_t>(static_cast<double>(sample_time_s) * 1e9);
    const std::int64_t offset_ns = static_cast<std::int64_t>(receive_ns) - sample_ns;
    // The freshest sample of each burst has the smallest latency; let the offset creep up to follow clock drift
    _clock_offset_ns = std::min(offset_ns, _clock_offset_ns == std::numeric_limits<std::int64_t>::max()
                                               ? offset_ns : _clock_offset_ns + 2000);
    return static_cast<std::uint64_t>(sample_ns + _clock_offset_ns);
}

void Odometry::_publish(const std::uint64_t timestamp_ns, const Eigen::Isometry2d& pose, const Eigen::Vector3d& twist) {
    if (timestamp_ns - _last_publish_ns < PUBLISH_PERIOD_NS) return;
    _last_publish_ns = timestamp_ns;

    core::publish_transform("odom", "base_link", timestamp_ns, core::geometry::to_pose3d(pose));

    // odom trail for viz, a point every 0.1 m
    const Eigen::Vector2f position = pose.translation().cast<float>();
    if (_trail.empty() || (position - _trail.back()).norm() > 0.1f) {
        _trail.push_back(position);
        if (_trail.size() > 10000) _trail.erase(_trail.begin());
    }
    if (timestamp_ns - _last_trail_render_ns >= TRAIL_RENDER_PERIOD_NS) {
        _last_trail_render_ns = timestamp_ns;
        std::vector<core::xyz_vec<float>> path;
        path.reserve(_trail.size());
        for (const Eigen::Vector2f& p : _trail) path.push_back({p.x(), p.y(), 0.0f});
        core::render_path(path, "odom_trail", "odom");
    }

    auto estimate = std::make_shared<dv_msgs::DriverlessStateEstimate>();
    estimate->set_timestamp_ns(timestamp_ns);
    estimate->set_initialized(true);
    estimate->set_x_odom_m(pose.translation().x());
    estimate->set_y_odom_m(pose.translation().y());
    estimate->set_yaw_odom_rad(core::geometry::yaw(pose));
    estimate->set_vx_vehicle_flu_mps(twist.x());
    estimate->set_vy_vehicle_flu_mps(twist.y());
    estimate->set_yaw_rate_vehicle_flu_radps(twist.z());
    core::log(estimate, "odometry");
}
