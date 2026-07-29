#include "ConfigParamLoader.hpp"

#include <cstddef>
#include <string>

#include <spdlog/spdlog.h>

#include "FoxgloveServer.hpp"

namespace app_config
{
namespace
{

template <typename ParamType>
ParamType config_param_or(const std::string& name, const ParamType default_value)
{
    if (const auto value =
            core::FoxgloveServer::instance().get_param<ParamType>(name))
    {
        return *value;
    }

    spdlog::warn("Using default config value for '{}'", name);
    return default_value;
}

double config_double_or(const std::string& name, const double default_value)
{
    if (const auto value = core::FoxgloveServer::instance().get_param<float>(name))
    {
        return static_cast<double>(*value);
    }

    if (const auto value = core::FoxgloveServer::instance().get_param<int>(name))
    {
        return static_cast<double>(*value);
    }

    spdlog::warn("Using default config value for '{}'", name);
    return default_value;
}

std::size_t config_size_or(const std::string& name,
                           const std::size_t default_value)
{
    if (const auto value = core::FoxgloveServer::instance().get_param<int>(name))
    {
        if (*value >= 0)
        {
            return static_cast<std::size_t>(*value);
        }

        spdlog::warn(
            "Using default config value for '{}' because configured value is "
            "negative",
            name);
        return default_value;
    }

    spdlog::warn("Using default config value for '{}'", name);
    return default_value;
}

}  // namespace

perception::LidarProcessorParams load_lidar_processor_params()
{
    perception::LidarProcessorParams params;

    params.deskew_enabled =
        config_param_or("LidarProcessor/deskew_enabled", params.deskew_enabled);

    auto& filter = params.point_cloud_filter_params;
    filter.angle_crop_enabled = config_param_or(
        "LidarProcessor/PointCloudFilter/angle_crop_enabled",
        filter.angle_crop_enabled);
    filter.roi_crop_enabled = config_param_or(
        "LidarProcessor/PointCloudFilter/roi_crop_enabled",
        filter.roi_crop_enabled);
    filter.angle_crop.min_angle_rad = config_double_or(
        "LidarProcessor/PointCloudFilter/min_angle_rad",
        filter.angle_crop.min_angle_rad);
    filter.angle_crop.max_angle_rad = config_double_or(
        "LidarProcessor/PointCloudFilter/max_angle_rad",
        filter.angle_crop.max_angle_rad);
    filter.roi.x_min_m = config_double_or(
        "LidarProcessor/PointCloudFilter/x_min_m", filter.roi.x_min_m);
    filter.roi.x_max_m = config_double_or(
        "LidarProcessor/PointCloudFilter/x_max_m", filter.roi.x_max_m);
    filter.roi.y_min_m = config_double_or(
        "LidarProcessor/PointCloudFilter/y_min_m", filter.roi.y_min_m);
    filter.roi.y_max_m = config_double_or(
        "LidarProcessor/PointCloudFilter/y_max_m", filter.roi.y_max_m);
    filter.roi.z_min_m = config_double_or(
        "LidarProcessor/PointCloudFilter/z_min_m", filter.roi.z_min_m);
    filter.roi.z_max_m = config_double_or(
        "LidarProcessor/PointCloudFilter/z_max_m", filter.roi.z_max_m);

    auto& ground = params.ground_removal_params;
    ground.grid_min_range_m = config_double_or(
        "LidarProcessor/GroundRemoval/grid_min_range_m",
        ground.grid_min_range_m);
    ground.grid_max_range_m = config_double_or(
        "LidarProcessor/GroundRemoval/grid_max_range_m",
        ground.grid_max_range_m);
    ground.grid_min_theta_rad = config_double_or(
        "LidarProcessor/GroundRemoval/grid_min_theta_rad",
        ground.grid_min_theta_rad);
    ground.grid_max_theta_rad = config_double_or(
        "LidarProcessor/GroundRemoval/grid_max_theta_rad",
        ground.grid_max_theta_rad);
    ground.flat_ground_z_max_m = config_double_or(
        "LidarProcessor/GroundRemoval/flat_ground_z_max_m",
        ground.flat_ground_z_max_m);
    ground.radial_bin_size_m = config_double_or(
        "LidarProcessor/GroundRemoval/radial_bin_size_m",
        ground.radial_bin_size_m);
    ground.angular_bin_size_rad = config_double_or(
        "LidarProcessor/GroundRemoval/angular_bin_size_rad",
        ground.angular_bin_size_rad);
    ground.ground_percentile = config_double_or(
        "LidarProcessor/GroundRemoval/ground_percentile",
        ground.ground_percentile);
    ground.min_points_per_cell = config_size_or(
        "LidarProcessor/GroundRemoval/min_points_per_cell",
        ground.min_points_per_cell);
    ground.non_ground_height_threshold_m = config_double_or(
        "LidarProcessor/GroundRemoval/non_ground_height_threshold_m",
        ground.non_ground_height_threshold_m);

    auto& clustering = params.clustering_params;
    clustering.cluster_tolerance_m = config_double_or(
        "LidarProcessor/Clustering/cluster_tolerance_m",
        clustering.cluster_tolerance_m);
    clustering.min_cluster_size = config_size_or(
        "LidarProcessor/Clustering/min_cluster_size",
        clustering.min_cluster_size);
    clustering.max_cluster_size = config_size_or(
        "LidarProcessor/Clustering/max_cluster_size",
        clustering.max_cluster_size);

    auto& cone = params.cone_filter_params;
    cone.max_detection_range_m = config_double_or(
        "LidarProcessor/ConeFilter/max_detection_range_m",
        cone.max_detection_range_m);
    cone.near_range_m = config_double_or("LidarProcessor/ConeFilter/near_range_m",
                                         cone.near_range_m);
    cone.mid_range_m = config_double_or("LidarProcessor/ConeFilter/mid_range_m",
                                        cone.mid_range_m);
    cone.near_min_cone_points = config_size_or(
        "LidarProcessor/ConeFilter/near_min_cone_points",
        cone.near_min_cone_points);
    cone.mid_min_cone_points = config_size_or(
        "LidarProcessor/ConeFilter/mid_min_cone_points",
        cone.mid_min_cone_points);
    cone.far_min_cone_points = config_size_or(
        "LidarProcessor/ConeFilter/far_min_cone_points",
        cone.far_min_cone_points);
    cone.near_min_cone_height_m = config_double_or(
        "LidarProcessor/ConeFilter/near_min_cone_height_m",
        cone.near_min_cone_height_m);
    cone.mid_min_cone_height_m = config_double_or(
        "LidarProcessor/ConeFilter/mid_min_cone_height_m",
        cone.mid_min_cone_height_m);
    cone.far_min_cone_height_m = config_double_or(
        "LidarProcessor/ConeFilter/far_min_cone_height_m",
        cone.far_min_cone_height_m);
    cone.near_max_cone_width_m = config_double_or(
        "LidarProcessor/ConeFilter/near_max_cone_width_m",
        cone.near_max_cone_width_m);
    cone.mid_max_cone_width_m = config_double_or(
        "LidarProcessor/ConeFilter/mid_max_cone_width_m",
        cone.mid_max_cone_width_m);
    cone.far_max_cone_width_m = config_double_or(
        "LidarProcessor/ConeFilter/far_max_cone_width_m",
        cone.far_max_cone_width_m);
    cone.max_cone_height_m = config_double_or(
        "LidarProcessor/ConeFilter/max_cone_height_m", cone.max_cone_height_m);
    cone.max_elongation_ratio = config_double_or(
        "LidarProcessor/ConeFilter/max_elongation_ratio",
        cone.max_elongation_ratio);
    cone.min_width_for_elongation_m = config_double_or(
        "LidarProcessor/ConeFilter/min_width_for_elongation_m",
        cone.min_width_for_elongation_m);
    cone.near_accepted_confidence = config_double_or(
        "LidarProcessor/ConeFilter/near_accepted_confidence",
        cone.near_accepted_confidence);
    cone.mid_accepted_confidence = config_double_or(
        "LidarProcessor/ConeFilter/mid_accepted_confidence",
        cone.mid_accepted_confidence);
    cone.far_accepted_confidence = config_double_or(
        "LidarProcessor/ConeFilter/far_accepted_confidence",
        cone.far_accepted_confidence);

    return params;
}

DriverlessEstimatorRunnerParams load_driverless_estimator_runner_params()
{
    DriverlessEstimatorRunnerParams params;

    params.ekf_params = estimation::EkfParams{0.1, 0.1, 0.5, 0.1, 0.1, 0.5};
    params.gss_sensor_config = estimation::GssSensorConfig{0.1, 0.1};

    auto& ekf = params.ekf_params;
    ekf.initial_position_std_m = config_double_or(
        "DriverlessEstimatorRunner/Ekf/initial_position_std_m",
        ekf.initial_position_std_m);
    ekf.initial_yaw_std_rad = config_double_or(
        "DriverlessEstimatorRunner/Ekf/initial_yaw_std_rad",
        ekf.initial_yaw_std_rad);
    ekf.initial_velocity_std_mps = config_double_or(
        "DriverlessEstimatorRunner/Ekf/initial_velocity_std_mps",
        ekf.initial_velocity_std_mps);
    ekf.position_process_std_m_per_sqrt_s = config_double_or(
        "DriverlessEstimatorRunner/Ekf/position_process_std_m_per_sqrt_s",
        ekf.position_process_std_m_per_sqrt_s);
    ekf.yaw_process_std_rad_per_sqrt_s = config_double_or(
        "DriverlessEstimatorRunner/Ekf/yaw_process_std_rad_per_sqrt_s",
        ekf.yaw_process_std_rad_per_sqrt_s);
    ekf.velocity_process_std_mps_per_sqrt_s = config_double_or(
        "DriverlessEstimatorRunner/Ekf/velocity_process_std_mps_per_sqrt_s",
        ekf.velocity_process_std_mps_per_sqrt_s);

    auto& gss = params.gss_sensor_config;
    gss.vx_noise_std_mps = config_double_or(
        "DriverlessEstimatorRunner/Gss/vx_noise_std_mps", gss.vx_noise_std_mps);
    gss.vy_noise_std_mps = config_double_or(
        "DriverlessEstimatorRunner/Gss/vy_noise_std_mps", gss.vy_noise_std_mps);

    return params;
}

}  // namespace app_config
