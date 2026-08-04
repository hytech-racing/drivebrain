#pragma once

#include "EstimatorTypes.hpp"
#include "LidarProcessor.hpp"
#include "RigidTransform3D.hpp"
#include "backend/IncrementalGraphSlamTypes.hpp"
#include "frontend/SlamFrontendTypes.hpp"

namespace app_config
{

struct DriverlessEstimatorRunnerParams
{
    estimation::EkfParams ekf_params;
    estimation::GssSensorConfig gss_sensor_config;
};

struct StaticTransformParams
{
    transforms::Pose3D T_base_imu;
    transforms::Pose3D T_base_gss;
    transforms::Pose3D T_base_lidar;
};

StaticTransformParams load_static_transform_params();

perception::LidarProcessorParams load_lidar_processor_params();

DriverlessEstimatorRunnerParams load_driverless_estimator_runner_params();

slam::frontend::SlamFrontendParams load_slam_frontend_params();

slam::backend::IncrementalGraphSlamParams load_incremental_graph_slam_params();

}  // namespace app_config
