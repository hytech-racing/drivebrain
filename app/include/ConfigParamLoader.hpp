#pragma once

#include "EstimatorTypes.hpp"
#include "LidarProcessor.hpp"

namespace app_config
{

struct DriverlessEstimatorRunnerParams
{
    estimation::EkfParams ekf_params;
    estimation::GssSensorConfig gss_sensor_config;
};

perception::LidarProcessorParams load_lidar_processor_params();

DriverlessEstimatorRunnerParams load_driverless_estimator_runner_params();

}  // namespace app_config
