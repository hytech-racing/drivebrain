#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>
#include <utility>

#include <Geometry.hpp>

namespace estimation {

/**
 * Time-ordered odom <- base_link poses with interpolated, thread-safe lookup
 */
class OdomBuffer {
public:
    explicit OdomBuffer(std::uint64_t history_duration_ns);

    /**
     * Rejects a zero or older timestamp or a non-finite pose. A repeated
     * timestamp replaces the newest sample.
     */
    bool insert(std::uint64_t timestamp_ns, const Eigen::Isometry2d& pose_odom_from_base);

    /**
     * Pose at the timestamp, interpolated between the surrounding samples.
     *
     * @param timeout how long to wait for a sample at or after the timestamp
     * @return nullopt if the timestamp is outside the buffered history
     */
    std::optional<Eigen::Isometry2d> lookup(std::uint64_t timestamp_ns,
                                            std::chrono::nanoseconds timeout = {}) const;

private:
    std::optional<Eigen::Isometry2d> _lookup_unlocked(std::uint64_t timestamp_ns) const;

    std::uint64_t _history_duration_ns;
    mutable std::mutex _mutex;
    mutable std::condition_variable _cv;
    std::deque<std::pair<std::uint64_t, Eigen::Isometry2d>> _buffer;
};

}  // namespace estimation
