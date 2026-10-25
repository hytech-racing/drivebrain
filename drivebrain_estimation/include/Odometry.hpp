#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

#include <Geometry.hpp>
#include "hytech_msgs.pb.h"

namespace estimation {

/**
 * Dead-reckoned odom <- base_link from the GSS
 */
class Odometry {
public:
    /* Reads the sensor mount position from the Odometry block of the config */
    Odometry();

    /**
     * Integrates one OMS sample, stamped by mapping its sample clock onto host time.
     * Velocity is zero at standstill and held when the sensor reports it invalid.
     *
     * @param receive_ns host time the sample arrived
     * @return false if the sample was not newer than the last one
     */
    bool update(const hytech_msgs::OMSData& sample, std::uint64_t receive_ns);

    /**
     * Pose at the timestamp, interpolated between the surrounding samples in the buffer
     *
     * @param timeout how long to wait for a sample at or after the timestamp
     * @return nullopt if the timestamp is outside the buffered history
     */
    std::optional<Eigen::Isometry2d> pose(std::uint64_t timestamp_ns,
                                          std::chrono::nanoseconds timeout = {}) const;

    /* Latest body velocity and yaw rate: (vx m/s, vy m/s, yaw rate rad/s) */
    Eigen::Vector3d twist() const;

private:
    /* Maps the OMS sample clock onto host time*/
    std::uint64_t _host_time_ns(float sample_time_s, std::uint64_t receive_ns);

    /* Publishes the odom -> base_link transform, the odom trail and the state estimate, rate-limited */
    void _publish(std::uint64_t timestamp_ns, const Eigen::Isometry2d& pose, const Eigen::Vector3d& twist);

    Eigen::Vector2d _position_base_m = Eigen::Vector2d::Zero();

    mutable std::mutex _mutex;
    mutable std::condition_variable _cv;
    std::deque<std::pair<std::uint64_t, Eigen::Isometry2d>> _buffer;
    Eigen::Isometry2d _pose = Eigen::Isometry2d::Identity();
    Eigen::Vector3d _twist = Eigen::Vector3d::Zero();
    std::uint64_t _timestamp_ns{};
    std::int64_t _clock_offset_ns = std::numeric_limits<std::int64_t>::max();

    std::uint64_t _last_publish_ns{};
    std::vector<Eigen::Vector2f> _trail;
    std::uint64_t _last_trail_render_ns{};
};

}  // namespace estimation
