#pragma once

#include "PointCloudTypes.hpp"

namespace perception
{

struct AngleCropConfig
{
    double min_angle_rad{-3.14159265358979323846};
    double max_angle_rad{3.14159265358979323846};
};

struct RoiConfig
{
    double x_min_m{0.0};
    double x_max_m{35.0};
    double y_min_m{-35.0};
    double y_max_m{35.0};
    double z_min_m{-0.65};
    double z_max_m{1.0};
};

struct PointCloudFilterParams
{
    bool angle_crop_enabled{true};
    bool roi_crop_enabled{true};

    AngleCropConfig angle_crop;
    RoiConfig roi;
};

[[nodiscard]] StampedPointCloud remove_non_finite(
    const StampedPointCloud& input);

[[nodiscard]] StampedPointCloud crop_by_angle(const StampedPointCloud& input,
                                              const AngleCropConfig& config);

[[nodiscard]] StampedPointCloud crop_by_roi(const StampedPointCloud& input,
                                            const RoiConfig& config);

[[nodiscard]] StampedPointCloud filter_point_cloud(
    const StampedPointCloud& input, const PointCloudFilterParams& params);

}  // namespace perception
