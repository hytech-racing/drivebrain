#include "OdomBuffer.h"

#include <algorithm>
#include <cmath>
#include <iterator>

using namespace estimation;

OdomBuffer::OdomBuffer(const std::uint64_t history_duration_ns)
    : _history_duration_ns(history_duration_ns) {}

bool OdomBuffer::insert(const std::uint64_t timestamp_ns, const Eigen::Isometry2d& pose_odom_from_base) {
    if (timestamp_ns == 0 || !pose_odom_from_base.matrix().allFinite()) {
        return false;
    }

    {
        std::scoped_lock lock(_mutex);

        if (!_buffer.empty()) {
            if (timestamp_ns < _buffer.back().first) {
                return false;
            }
            if (timestamp_ns == _buffer.back().first) {
                _buffer.pop_back();
            }
        }
        _buffer.emplace_back(timestamp_ns, pose_odom_from_base);

        // Always keeps the newest sample
        while (_buffer.size() > 1 &&
               _buffer.back().first - _buffer.front().first > _history_duration_ns) {
            _buffer.pop_front();
        }
    }

    _cv.notify_all();
    return true;
}

std::optional<Eigen::Isometry2d> OdomBuffer::lookup(const std::uint64_t timestamp_ns,
                                                    const std::chrono::nanoseconds timeout) const {
    std::unique_lock lock(_mutex);

    if (timeout > std::chrono::nanoseconds{0}) {
        // A timestamp older than the buffer will never become available
        _cv.wait_for(lock, timeout, [&] {
            return !_buffer.empty() && (timestamp_ns <= _buffer.back().first ||
                                        timestamp_ns < _buffer.front().first);
        });
    }

    return _lookup_unlocked(timestamp_ns);
}

std::optional<Eigen::Isometry2d> OdomBuffer::_lookup_unlocked(const std::uint64_t timestamp_ns) const {
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
