#pragma once

#include <cstdint>
#include <deque>
#include <mutex>
#include <optional>

#include "FrameId.hpp"
#include "RigidTransform2D.hpp"

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
                            const RigidTransform2D& transform);

    /**
     * Point definition: T_map_odom transforms coordinates expressed in odom
     * into coordinates expressed in map
     *
     * Pose definition: T_map_odom represents the pose of odom
     * expresesd in map
     */
    bool insert_T_map_odom(const std::uint64_t timestamp_ns,
                           const RigidTransform2D& transform);

    /**
     * Sets the static transform T_base_imu
     *
     * Point definition: T_base_imu transforms coordinates expressed in
     * imu into coordinates expressed in base_link
     *
     * Pose definition: T_base_imu represents the pose of the IMU
     * expressed in base_link
     */
    bool set_base_to_imu(const RigidTransform2D& transform);

    /**
     * Sets the static transform T_base_gss
     *
     * Point definition: T_base_gss transforms coordinates expressed in
     * gss into coordinates expressed in base_link
     *
     * Pose definition: T_base_gss represents the pose of the GSS
     * expressed in base_link
     */
    bool set_base_to_gss(const RigidTransform2D& transform);

    /**
     * Sets the static transform T_base_lidar
     *
     * Point definition: T_base_imu transforms coordinates expressed in
     * lidar into coordinates expressed in base_link
     *
     * Pose definition: T_base_lidar represents the pose of the lidar
     * expressed in base_link
     */
    bool set_base_to_lidar(const RigidTransform2D& transform);

    /**
     * Looks up the transform T_target_source at query_timestamp_ns
     *
     * Point definition: T_target_source transforms coordinates expressed in
     * source into coordinates expressed in target
     *
     * Pose definition: T_target_source represents the pose of source
     * expressed in target
     */
    std::optional<RigidTransform2D> lookup(
        const FrameId target, const FrameId source,
        const std::uint64_t query_timestamp_ns) const;

   private:
    void _remove_stale_transforms(
        std::deque<std::pair<RigidTransform2D, std::uint64_t>>& buffer);

    bool _transform_is_finite(const RigidTransform2D& transform) const;

    bool _timestamp_out_of_buffer_bound(
        const std::deque<std::pair<RigidTransform2D, std::uint64_t>>& buffer,
        std::uint64_t query_timestamp_ns) const;

    std::optional<RigidTransform2D> _lookup_T_odom_base_unlocked(
        std::uint64_t query_timestamp_ns) const;

    std::optional<RigidTransform2D> _lookup_T_map_odom_unlocked(
        std::uint64_t query_timestamp_ns) const;

    std::optional<RigidTransform2D> _get_T_odom_frame_unlocked(
        const FrameId frame, const std::uint64_t timestamp_ns) const;

   private:
    std::uint64_t _history_duration_ns;

    mutable std::mutex _mutex;

    RigidTransform2D _T_base_imu{};
    RigidTransform2D _T_base_gss{};
    RigidTransform2D _T_base_lidar{};

    std::deque<std::pair<RigidTransform2D, std::uint64_t>> _T_odom_base_buffer;
    std::deque<std::pair<RigidTransform2D, std::uint64_t>> _T_map_odom_buffer;
};
}  // namespace transforms
