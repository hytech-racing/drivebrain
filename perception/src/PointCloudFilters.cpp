#include "PointCloudFilters.hpp"

#include <cmath>

namespace perception
{

StampedPointCloud remove_non_finite(const StampedPointCloud& input)
{
    StampedPointCloud output;
    output.timestamp_ns = input.timestamp_ns;
    output.frame = input.frame;
    output.points.reserve(input.points.size());

    for (const PointXYZI& point : input.points)
    {
        if (std::isfinite(point.x) && std::isfinite(point.y) &&
            std::isfinite(point.z) && std::isfinite(point.intensity))
        {
            output.points.emplace_back(point);
        }
    }

    return output;
}

StampedPointCloud crop_by_angle(const StampedPointCloud& input,
                                const AngleCropConfig& config)
{
    StampedPointCloud output;
    output.timestamp_ns = input.timestamp_ns;
    output.frame = input.frame;
    output.points.reserve(input.points.size());

    for (const PointXYZI& point : input.points)
    {
        const double angle_rad = std::atan2(point.y, point.x);

        if (angle_rad >= config.min_angle_rad &&
            angle_rad <= config.max_angle_rad)
        {
            output.points.emplace_back(point);
        }
    }

    return output;
}

StampedPointCloud crop_by_roi(const StampedPointCloud& input,
                              const RoiConfig& config)
{
    StampedPointCloud output;
    output.timestamp_ns = input.timestamp_ns;
    output.frame = input.frame;
    output.points.reserve(input.points.size());

    for (const PointXYZI& point : input.points)
    {
        const bool x_check =
            point.x >= config.x_min_m && point.x <= config.x_max_m;
        const bool y_check =
            point.y >= config.y_min_m && point.y <= config.y_max_m;
        const bool z_check =
            point.z >= config.z_min_m && point.z <= config.z_max_m;

        if (x_check && y_check && z_check)
        {
            output.points.emplace_back(point);
        }
    }

    return output;
}

StampedPointCloud filter_point_cloud(const StampedPointCloud& input,
                                     const PointCloudFilterParams& params)
{
    StampedPointCloud filtered = remove_non_finite(input);

    if (params.angle_crop_enabled)
    {
        filtered = crop_by_angle(filtered, params.angle_crop);
    }

    if (params.roi_crop_enabled)
    {
        filtered = crop_by_roi(filtered, params.roi);
    }

    return filtered;
}

}  // namespace perception
