
#include "PointCloudDeskew.hpp"

#include <stdexcept>

namespace perception
{

PointXYZI transform_point(const transforms::Pose3D& transform,
                          const PointXYZI& point) noexcept
{
    const transforms::Quaternion& q = transform.q;

    double num1 = q.x * 2.0;
    double num2 = q.y * 2.0;
    double num3 = q.z * 2.0;
    double num4 = q.x * num1;
    double num5 = q.y * num2;
    double num6 = q.z * num3;
    double num7 = q.x * num2;
    double num8 = q.x * num3;
    double num9 = q.y * num3;
    double num10 = q.w * num1;
    double num11 = q.w * num2;
    double num12 = q.w * num3;

    double trans_x =
        transform.x_m + (1.0 - (num5 + num6)) * point.x +
        (num7 - num12) * point.y + (num8 + num11) * point.z;
    double trans_y =
        transform.y_m + (num7 + num12) * point.x +
        (1.0 - (num4 + num6)) * point.y + (num9 - num10) * point.z;
    double trans_z =
        transform.z_m + (num8 - num11) * point.x +
        (num9 + num10) * point.y + (1.0 - (num4 + num5)) * point.z;

    return PointXYZI{static_cast<float>(trans_x),
                     static_cast<float>(trans_y),
                     static_cast<float>(trans_z), point.intensity};
}

DeskewResult deskew_point_cloud(const StampedPointCloud& stamped_point_cloud,
                                const StampedLidarPose& T_odom_reference,
                                const std::vector<StampedLidarPose>& T_odom_i)
{
    DeskewResult result;

    if (T_odom_i.size() != stamped_point_cloud.points.size())
    {
        throw std::invalid_argument("Point and pose counts must match");
    }

    result.end_stamp_ns = stamped_point_cloud.timestamp_ns;
    result.reference_stamp_ns = T_odom_reference.stamp_ns;
    result.stamped_point_cloud.timestamp_ns = result.reference_stamp_ns;

    if (stamped_point_cloud.points.empty())
    {
        result.start_stamp_ns = T_odom_reference.stamp_ns;
        return result;
    }

    std::size_t total_points = stamped_point_cloud.points.size();

    result.stamped_point_cloud.points.reserve(total_points);

    for (std::size_t i = 0; i < stamped_point_cloud.points.size(); ++i)
    {
        const transforms::Pose3D T_to_ref_from_i =
            T_odom_reference.pose.inverse().compose(T_odom_i[i].pose);

        result.stamped_point_cloud.points.push_back(
            transform_point(T_to_ref_from_i, stamped_point_cloud.points[i]));
    }

    result.start_stamp_ns = T_odom_i.front().stamp_ns;

    return result;
}

}  // namespace perception
