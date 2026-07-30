#include "TransformBuffer.hpp"

#include <algorithm>
#include <cmath>

namespace transforms
{

namespace
{
double lerp(double start, double end, double alpha)
{
    return start + alpha * (end - start);
}

double wrap_to_pi(double theta)
{
    constexpr double pi = 3.14159265358979323846;
    constexpr double two_pi = 2.0 * pi;

    theta = std::fmod(theta + pi, two_pi);
    if (theta < 0.0)
    {
        theta += two_pi;
    }
    return theta - pi;
}

Pose3D to_pose3d(const Pose2D& pose)
{
    const double half_yaw_rad = 0.5 * pose.yaw_rad;
    return Pose3D{
        pose.x_m, pose.y_m, 0.0,
        Quaternion{std::cos(half_yaw_rad), 0.0, 0.0, std::sin(half_yaw_rad)}};
}

}  // namespace

TransformBuffer::TransformBuffer(const std::uint64_t history_duration_ns)
    : _history_duration_ns(history_duration_ns)
{
}

bool TransformBuffer::insert_T_odom_base(const std::uint64_t timestamp_ns,
                                         const Pose2D& transform)
{
    {
        std::scoped_lock lock(_mutex);

        if (timestamp_ns == 0 || !_transform_is_finite(transform))
        {
            return false;
        }

        if (_T_odom_base_buffer.empty())
        {
            _T_odom_base_buffer.emplace_back(transform, timestamp_ns);
        }
        else
        {
            const std::uint64_t latest_stamp_ns =
                _T_odom_base_buffer.back().second;
            if (timestamp_ns < latest_stamp_ns)
            {
                return false;
            }

            if (timestamp_ns == latest_stamp_ns)
            {
                _T_odom_base_buffer.pop_back();
            }

            _T_odom_base_buffer.emplace_back(transform, timestamp_ns);
            _remove_stale_transforms(_T_odom_base_buffer);
        }
    }

    _cv.notify_all();
    return true;
}

bool TransformBuffer::insert_T_map_odom(const std::uint64_t timestamp_ns,
                                        const Pose2D& transform)
{
    {
        std::scoped_lock lock(_mutex);

        if (timestamp_ns == 0 || !_transform_is_finite(transform))
        {
            return false;
        }

        if (_T_map_odom_buffer.empty())
        {
            _T_map_odom_buffer.emplace_back(transform, timestamp_ns);
        }
        else
        {
            const std::uint64_t latest_stamp_ns =
                _T_map_odom_buffer.back().second;
            if (timestamp_ns < latest_stamp_ns)
            {
                return false;
            }

            if (timestamp_ns == latest_stamp_ns)
            {
                _T_map_odom_buffer.pop_back();
            }

            _T_map_odom_buffer.emplace_back(transform, timestamp_ns);
            _remove_stale_transforms(_T_map_odom_buffer);
        }
    }

    _cv.notify_all();
    return true;
}

bool TransformBuffer::set_T_base_imu(const Pose2D& transform)
{
    std::scoped_lock lock(_mutex);

    if (!_transform_is_finite(transform))
    {
        return false;
    }

    _T_base_imu = transform;
    return true;
}

bool TransformBuffer::set_T_base_gss(const Pose2D& transform)
{
    std::scoped_lock lock(_mutex);

    if (!_transform_is_finite(transform))
    {
        return false;
    }

    _T_base_gss = transform;
    return true;
}

bool TransformBuffer::set_T_base_lidar(const Pose2D& transform)
{
    std::scoped_lock lock(_mutex);

    if (!_transform_is_finite(transform))
    {
        return false;
    }

    _T_base_lidar = transform;
    return true;
}

Pose2D TransformBuffer::T_base_imu() const
{
    std::scoped_lock lock(_mutex);
    return _T_base_imu;
}

Pose2D TransformBuffer::T_base_gss() const
{
    std::scoped_lock lock(_mutex);
    return _T_base_gss;
}

Pose2D TransformBuffer::T_base_lidar() const
{
    std::scoped_lock lock(_mutex);
    return _T_base_lidar;
}

Pose3D TransformBuffer::T_base_imu3d() const
{
    std::scoped_lock lock(_mutex);
    return to_pose3d(_T_base_imu);
}

Pose3D TransformBuffer::T_base_gss3d() const
{
    std::scoped_lock lock(_mutex);
    return to_pose3d(_T_base_gss);
}

Pose3D TransformBuffer::T_base_lidar3d() const
{
    std::scoped_lock lock(_mutex);
    return to_pose3d(_T_base_lidar);
}

void TransformBuffer::_remove_stale_transforms(
    std::deque<std::pair<Pose2D, std::uint64_t>>& buffer)
{
    if (buffer.empty())
    {
        return;
    }

    const std::uint64_t latest_stamp_ns = buffer.back().second;

    while (buffer.size() > 1 &&
           latest_stamp_ns - buffer.front().second > _history_duration_ns)
    {
        buffer.pop_front();
    }
}

std::optional<Pose2D> TransformBuffer::_lookup_T_odom_base_unlocked(
    std::uint64_t query_timestamp_ns) const
{
    if (_timestamp_out_of_buffer_bound(_T_odom_base_buffer, query_timestamp_ns))
    {
        return std::nullopt;
    }

    auto it =
        std::lower_bound(_T_odom_base_buffer.begin(), _T_odom_base_buffer.end(),
                         query_timestamp_ns,
                         [](const std::pair<Pose2D, std::uint64_t>& element,
                            std::uint64_t query_stamp_ns)
                         { return element.second < query_stamp_ns; });

    if (it->second == query_timestamp_ns)
    {
        return it->first;
    }

    const auto it_after = it;
    const auto it_before = std::prev(it);

    const std::uint64_t t0 = it_before->second;
    const std::uint64_t t1 = it_after->second;

    const double alpha = static_cast<double>(query_timestamp_ns - t0) /
                         static_cast<double>(t1 - t0);

    const double delta_yaw =
        wrap_to_pi(it_after->first.yaw_rad - it_before->first.yaw_rad);
    const double interpolated_yaw =
        wrap_to_pi(it_before->first.yaw_rad + alpha * delta_yaw);

    return Pose2D{lerp(it_before->first.x_m, it_after->first.x_m, alpha),
                  lerp(it_before->first.y_m, it_after->first.y_m, alpha),
                  interpolated_yaw};
}

// Does not interpolate T_map_odom, instead, returns latest T_map_odom within
// bound
std::optional<Pose2D> TransformBuffer::_lookup_T_map_odom_unlocked(
    std::uint64_t query_timestamp_ns) const
{
    if (_timestamp_out_of_buffer_bound(_T_map_odom_buffer, query_timestamp_ns))
    {
        return std::nullopt;
    }

    const auto it_after =
        std::upper_bound(_T_map_odom_buffer.begin(), _T_map_odom_buffer.end(),
                         query_timestamp_ns,
                         [](std::uint64_t query_stamp_ns,
                            const std::pair<Pose2D, std::uint64_t>& element)
                         { return query_stamp_ns < element.second; });

    return std::prev(it_after)->first;
}

std::optional<Pose2D> TransformBuffer::_get_T_odom_frame_unlocked(
    const FrameId frame, const std::uint64_t timestamp_ns) const
{
    Pose2D T_base_sensor;
    bool is_static_sensor = true;

    switch (frame)
    {
        case FrameId::Baselink:
            T_base_sensor = Pose2D{0.0, 0.0, 0.0};
            break;
        case FrameId::Imu:
            T_base_sensor = _T_base_imu;
            break;
        case FrameId::Gss:
            T_base_sensor = _T_base_gss;
            break;
        case FrameId::Lidar:
            T_base_sensor = _T_base_lidar;
            break;
        default:
            is_static_sensor = false;
            break;
    }

    if (is_static_sensor)
    {
        // T_odom_sensor = T_odom_base * T_base_sensor
        std::optional<Pose2D> T_odom_base =
            _lookup_T_odom_base_unlocked(timestamp_ns);
        if (!T_odom_base) return std::nullopt;

        return (*T_odom_base) * T_base_sensor;
    }

    if (frame == FrameId::Odom)
    {
        return Pose2D{0.0, 0.0, 0.0};  // Identity
    }

    return std::nullopt;
}

std::optional<Pose2D> TransformBuffer::lookup(
    const FrameId target, const FrameId source,
    const std::uint64_t timestamp_ns, std::chrono::nanoseconds timeout) const
{
    std::unique_lock lock(_mutex);

    if (timeout > std::chrono::nanoseconds{0})
    {
        _cv.wait_for(
            lock, timeout, [this, target, source, timestamp_ns]()
            { return _lookup_ready_unlocked(target, source, timestamp_ns); });
    }

    return _lookup_unlocked(target, source, timestamp_ns);
}

std::optional<Pose2D> TransformBuffer::_lookup_unlocked(
    const FrameId target, const FrameId source,
    const std::uint64_t timestamp_ns) const
{
    if (target == source)
    {
        return Pose2D{0.0, 0.0, 0.0};
    }

    // If neither frame is map, then we can use the higher-frequency T_odom_base
    // buffer
    if (target != FrameId::Map && source != FrameId::Map)
    {
        auto T_odom_target = _get_T_odom_frame_unlocked(target, timestamp_ns);
        auto T_odom_source = _get_T_odom_frame_unlocked(source, timestamp_ns);

        if (!T_odom_target || !T_odom_source)
        {
            return std::nullopt;
        }

        return T_odom_target->inverse() * (*T_odom_source);
    }

    // If one of the frames is map, then we'll need the lower-frequency SLAM
    // supplied T_map_odom buffer
    auto T_map_odom = _lookup_T_map_odom_unlocked(timestamp_ns);

    if (!T_map_odom) return std::nullopt;

    if (target == FrameId::Map)
    {
        // source is something like lidar or base_link
        auto T_odom_source = _get_T_odom_frame_unlocked(source, timestamp_ns);

        if (!T_odom_source)
        {
            return std::nullopt;
        }

        return (*T_map_odom) * (*T_odom_source);
    }
    else  // source == FrameId::Map
    {
        // target is something like lidar or base_link
        auto T_odom_target = _get_T_odom_frame_unlocked(target, timestamp_ns);

        if (!T_odom_target)
        {
            return std::nullopt;
        }

        // T_target_map = (T_map_odom * T_odom_target)^-1
        return ((*T_map_odom) * (*T_odom_target)).inverse();
    }
}

std::optional<Pose3D> TransformBuffer::lookup3d(
    const FrameId target, const FrameId source,
    const std::uint64_t timestamp_ns, std::chrono::nanoseconds timeout) const
{
    const std::optional<Pose2D> pose2d =
        lookup(target, source, timestamp_ns, timeout);
    if (!pose2d)
    {
        return std::nullopt;
    }

    return to_pose3d(*pose2d);
}

bool TransformBuffer::_lookup_ready_unlocked(
    const FrameId target, const FrameId source,
    const std::uint64_t timestamp_ns) const
{
    if (target == source)
    {
        return true;
    }

    const bool needs_odom_buffer = _frame_requires_odom_buffer(target) ||
                                   _frame_requires_odom_buffer(source);
    const bool needs_map_odom_buffer =
        target == FrameId::Map || source == FrameId::Map;

    if (needs_odom_buffer)
    {
        if (_T_odom_base_buffer.empty())
        {
            return false;
        }

        if (timestamp_ns < _T_odom_base_buffer.front().second ||
            timestamp_ns > _T_odom_base_buffer.back().second)
        {
            return timestamp_ns < _T_odom_base_buffer.front().second;
        }
    }

    if (needs_map_odom_buffer)
    {
        if (_T_map_odom_buffer.empty())
        {
            return false;
        }

        if (timestamp_ns < _T_map_odom_buffer.front().second ||
            timestamp_ns > _T_map_odom_buffer.back().second)
        {
            return timestamp_ns < _T_map_odom_buffer.front().second;
        }
    }

    return true;
}

bool TransformBuffer::_frame_requires_odom_buffer(const FrameId frame) const
{
    return frame == FrameId::Baselink || frame == FrameId::Imu ||
           frame == FrameId::Gss || frame == FrameId::Lidar;
}

bool TransformBuffer::_transform_is_finite(const Pose2D& transform) const
{
    return std::isfinite(transform.x_m) && std::isfinite(transform.y_m) &&
           std::isfinite(transform.yaw_rad);
}

bool TransformBuffer::_timestamp_out_of_buffer_bound(
    const std::deque<std::pair<Pose2D, std::uint64_t>>& buffer,
    std::uint64_t query_timestamp_ns) const
{
    if (buffer.empty())
    {
        return true;
    }

    return query_timestamp_ns < buffer.front().second ||
           query_timestamp_ns > buffer.back().second;
}

std::uint64_t TransformBuffer::latest_odom_buffer_timestamp_ns() const
{
    std::scoped_lock lock(_mutex);

    if (_T_odom_base_buffer.empty())
    {
        return 0;
    }

    return _T_odom_base_buffer.back().second;
}
}  // namespace transforms
