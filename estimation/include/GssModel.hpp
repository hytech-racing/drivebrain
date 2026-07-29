
#pragma once
#include "EstimatorMeasurements.hpp"
#include "EstimatorTypes.hpp"

namespace estimation
{

[[nodiscard]] GssMeasurementEigen predict_gss_measurement(
    const InternalEstimatorState& state, double yaw_rate_vehicle_flu_radps,
    const transforms::Pose2D& T_base_gss);

[[nodiscard]] GssMeasurementJacobian compute_gss_measurement_jacobian(
    const transforms::Pose2D& T_base_gss);

}  // namespace estimation
