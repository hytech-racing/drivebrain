
#include "GssModel.hpp"

namespace estimation
{

[[nodiscard]] GssMeasurementEigen predict_gss_measurement(
    const InternalEstimatorState& state, double yaw_rate_vehicle_flu_radps,
    const transforms::Pose2D& T_base_gss)
{
    GssMeasurementEigen gss_meas;

    gss_meas.vx_sensor_mps() =
        state.vx_body_mps() - yaw_rate_vehicle_flu_radps * T_base_gss.y_m;
    gss_meas.vy_sensor_mps() =
        state.vy_body_mps() + yaw_rate_vehicle_flu_radps * T_base_gss.x_m;

    return gss_meas;
}

[[nodiscard]] GssMeasurementJacobian compute_gss_measurement_jacobian(
    const transforms::Pose2D& T_base_gss)
{
    GssMeasurementJacobian gss_jacobian = GssMeasurementJacobian::Zero();

    gss_jacobian(GssMeasurementIndex::VX_SENSOR, StateIndex::VX_BODY) = 1.0;
    gss_jacobian(GssMeasurementIndex::VY_SENSOR, StateIndex::VY_BODY) = 1.0;

    return gss_jacobian;
}

}  // namespace estimation
