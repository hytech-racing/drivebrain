#pragma once

#include <optional>

#include "EstimatorMeasurements.hpp"
#include "hytech_msgs.pb.h"

namespace adapters
{

std::optional<estimation::ImuMeasurement> to_imu_measurement(
    const hytech_msgs::VnImuData& message);

std::optional<estimation::GssMeasurement> to_gss_measurement(
    const hytech_msgs::GssData& message);

}  // namespace adapters
