#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>

#include "FrameId.hpp"
#include "geometry/Pose2D.hpp"
#include "geometry/Pose3D.hpp"

namespace transforms
{
/**
 * Everything inside the TransformBuffer class expects adherence to FLU axis
 * system
 */
class TransformBuffer
{
   public:
    TransformBuffer(const std::uint64_t history_duration_ns);

    /**
     * Point definition: T_odom_base transforms coordinates expressed in
     * base_link into coordinates expressed in odom
     *
     * Pose definition: T_odom_base represents the pose of base_link
     * expresesd in odom
     */
    bool insert_T_odom_base(const std::uint64_t timestamp_ns,
                            const core::geometry::Pose2D& transform);

    bool insert_T_odom_base3d(const std::uint64_t timestamp_ns,
                              const core::geometry::Pose3D& transform);

    /**
     * Point definition: T_map_odom transforms coordinates expressed in odom
     * into coordinates expressed in map
     *
     * Pose definition: T_map_odom represents the pose of odom
     * expresesd in map
     */
    bool insert_T_map_odom(const std::uint64_t timestamp_ns,
                           const core::geometry::Pose2D& transform);

    bool insert_T_map_odom3d(const std::uint64_t timestamp_ns,
                             const core::geometry::Pose3D& transform);

    /**
     * Sets the static transform T_base_imu
     *
     * Point definition: T_base_imu transforms coordinates expressed in
     * imu into coordinates expressed in base_link
     *
     * Pose definition: T_base_imu represents the pose of the IMU
     * expressed in base_link
     */
    bool set_T_base_imu(const core::geometry::Pose2D& transform);

    bool set_T_base_imu3d(const core::geometry::Pose3D& transform);

    /**
     * Sets the static transform T_base_gss
     *
     * Point definition: T_base_gss transforms coordinates expressed in
     * gss into coordinates expressed in base_link
     *
     * Pose definition: T_base_gss represents the pose of the GSS
     * expressed in base_link
     */
    bool set_T_base_gss(const core::geometry::Pose2D& transform);

    bool set_T_base_gss3d(const core::geometry::Pose3D& transform);

    /**
     * Sets the static transform T_base_lidar
     *
     * Point definition: T_base_imu transforms coordinates expressed in
     * lidar into coordinates expressed in base_link
     *
     * Pose definition: T_base_lidar represents the pose of the lidar
     * expressed in base_link
     */
    bool set_T_base_lidar(const core::geometry::Pose2D& transform);

    bool set_T_base_lidar3d(const core::geometry::Pose3D& transform);

    bool set_T_base_camera_wide3d(const core::geometry::Pose3D& transform);

    bool set_T_base_camera_narrow3d(const core::geometry::Pose3D& transform);

    [[nodiscard]] core::geometry::Pose2D T_base_imu() const;

    [[nodiscard]] core::geometry::Pose2D T_base_gss() const;

    [[nodiscard]] core::geometry::Pose2D T_base_lidar() const;

    [[nodiscard]] core::geometry::Pose3D T_base_imu3d() const;

    [[nodiscard]] core::geometry::Pose3D T_base_gss3d() const;

    [[nodiscard]] core::geometry::Pose3D T_base_lidar3d() const;

    [[nodiscard]] core::geometry::Pose3D T_base_camera_wide3d() const;

    [[nodiscard]] core::geometry::Pose3D T_base_camera_narrow3d() const;

    /**
     * Looks up the transform T_target_source at query_timestamp_ns
     *
     * Point definition: T_target_source transforms coordinates expressed in
     * source into coordinates expressed in target
     *
     * Pose definition: T_target_source represents the pose of source
     * expressed in target
     */
    std::optional<core::geometry::Pose2D> lookup(
        const FrameId target, const FrameId source,
        const std::uint64_t query_timestamp_ns,
        std::chrono::nanoseconds timeout = std::chrono::nanoseconds{0}) const;

    std::optional<core::geometry::Pose3D> lookup3d(
        const FrameId target, const FrameId source,
        const std::uint64_t query_timestamp_ns,
        std::chrono::nanoseconds timeout = std::chrono::nanoseconds{0}) const;

    std::uint64_t latest_odom_buffer_timestamp_ns() const;

   private:
    void _remove_stale_transforms(
        std::deque<std::pair<core::geometry::Pose3D, std::uint64_t>>& buffer);

    bool _transform_is_finite(const core::geometry::Pose3D& transform) const;

    bool _timestamp_out_of_buffer_bound(
        const std::deque<std::pair<core::geometry::Pose3D, std::uint64_t>>& buffer,
        std::uint64_t query_timestamp_ns) const;

    std::optional<core::geometry::Pose3D> _lookup_T_odom_base_unlocked(
        std::uint64_t query_timestamp_ns) const;

    std::optional<core::geometry::Pose3D> _lookup_T_map_odom_unlocked(
        std::uint64_t query_timestamp_ns) const;

    std::optional<core::geometry::Pose3D> _get_T_odom_frame_unlocked(
        const FrameId frame, const std::uint64_t timestamp_ns) const;

    std::optional<core::geometry::Pose3D> _lookup_unlocked(
        const FrameId target, const FrameId source,
        const std::uint64_t timestamp_ns) const;

    bool _lookup_ready_unlocked(const FrameId target, const FrameId source,
                                const std::uint64_t timestamp_ns) const;

    bool _frame_requires_odom_buffer(const FrameId frame) const;

   private:
    std::uint64_t _history_duration_ns;

    mutable std::mutex _mutex;
    mutable std::condition_variable _cv;

    core::geometry::Pose3D _T_base_imu{};
    core::geometry::Pose3D _T_base_gss{};
    core::geometry::Pose3D _T_base_lidar{};
    core::geometry::Pose3D _T_base_camera_wide{};
    core::geometry::Pose3D _T_base_camera_narrow{};

    std::deque<std::pair<core::geometry::Pose3D, std::uint64_t>> _T_odom_base_buffer;
    std::deque<std::pair<core::geometry::Pose3D, std::uint64_t>> _T_map_odom_buffer;
};
}  // namespace transforms
