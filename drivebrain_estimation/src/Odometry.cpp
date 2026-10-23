#include "Odometry.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include <FoxgloveServer.hpp>

using namespace estimation;

Odometry::Odometry(const std::uint64_t history_duration_ns)
    : _history_duration_ns(history_duration_ns) {
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
               _buffer.back().first - _buffer.front().first > _history_duration_ns) {
            _buffer.pop_front();
        }
    }

    _cv.notify_all();
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

std::pair<std::uint64_t, Eigen::Isometry2d> Odometry::latest() const {
    std::scoped_lock lock(_mutex);
    return {_timestamp_ns, _pose};
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
