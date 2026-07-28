
#include "MotionModel.hpp"

#include <cmath>

#include "EstimatorTypes.hpp"

namespace estimation
{

InternalEstimatorState predict_nominal_state(
    const InternalEstimatorState& previous_state, const ImuMeasurement& input,
    const double dt_s)
{
    InternalEstimatorState current_state = previous_state;

    const double cy = std::cos(previous_state.yaw_odom_rad());
    const double sy = std::sin(previous_state.yaw_odom_rad());

    current_state.x_odom_m() +=
        dt_s * (previous_state.vx_body_mps() * cy -
                previous_state.vy_body_mps() * sy) +
        0.5 *
            (input.ax_vehicle_flu_mps2 +
             previous_state.vy_body_mps() * input.yaw_rate_vehicle_flu_radps) *
            dt_s * dt_s * cy -
        0.5 *
            (input.ay_vehicle_flu_mps2 -
             previous_state.vx_body_mps() * input.yaw_rate_vehicle_flu_radps) *
            dt_s * dt_s * sy;

    current_state.y_odom_m() +=
        dt_s * (previous_state.vx_body_mps() * sy +
                previous_state.vy_body_mps() * cy) +
        0.5 *
            (input.ax_vehicle_flu_mps2 +
             previous_state.vy_body_mps() * input.yaw_rate_vehicle_flu_radps) *
            dt_s * dt_s * sy +
        0.5 *
            (input.ay_vehicle_flu_mps2 -
             previous_state.vx_body_mps() * input.yaw_rate_vehicle_flu_radps) *
            dt_s * dt_s * cy;

    current_state.yaw_odom_rad() += dt_s * input.yaw_rate_vehicle_flu_radps;

    current_state.vx_body_mps() +=
        (input.ax_vehicle_flu_mps2 +
         previous_state.vy_body_mps() * input.yaw_rate_vehicle_flu_radps) *
        dt_s;

    current_state.vy_body_mps() +=
        (input.ay_vehicle_flu_mps2 -
         previous_state.vx_body_mps() * input.yaw_rate_vehicle_flu_radps) *
        dt_s;

    return current_state;
}

StateJacobian compute_state_transition_jacobian(
    const InternalEstimatorState& previous_state, const ImuMeasurement& input,
    const double dt_s)
{
    StateJacobian state_jacobian = StateJacobian::Identity();

    const double cy = std::cos(previous_state.yaw_odom_rad());
    const double sy = std::sin(previous_state.yaw_odom_rad());
    const double wz = input.yaw_rate_vehicle_flu_radps;
    const double dt2 = dt_s * dt_s;
    const double ax_body =
        input.ax_vehicle_flu_mps2 + previous_state.vy_body_mps() * wz;
    const double ay_body =
        input.ay_vehicle_flu_mps2 - previous_state.vx_body_mps() * wz;

    // Position row updates
    state_jacobian(StateIndex::X_ODOM, StateIndex::YAW_ODOM) =
        dt_s * (-previous_state.vx_body_mps() * sy -
                previous_state.vy_body_mps() * cy) +
        0.5 * dt2 * (-ax_body * sy - ay_body * cy);
    state_jacobian(StateIndex::X_ODOM, StateIndex::VX_BODY) =
        dt_s * cy + 0.5 * dt2 * wz * sy;
    state_jacobian(StateIndex::X_ODOM, StateIndex::VY_BODY) =
        -dt_s * sy + 0.5 * dt2 * wz * cy;

    state_jacobian(StateIndex::Y_ODOM, StateIndex::YAW_ODOM) =
        dt_s * (previous_state.vx_body_mps() * cy -
                previous_state.vy_body_mps() * sy) +
        0.5 * dt2 * (ax_body * cy - ay_body * sy);
    state_jacobian(StateIndex::Y_ODOM, StateIndex::VX_BODY) =
        dt_s * sy - 0.5 * dt2 * wz * cy;
    state_jacobian(StateIndex::Y_ODOM, StateIndex::VY_BODY) =
        dt_s * cy + 0.5 * dt2 * wz * sy;

    // Velocity row updates
    state_jacobian(StateIndex::VX_BODY, StateIndex::VY_BODY) = wz * dt_s;
    state_jacobian(StateIndex::VY_BODY, StateIndex::VX_BODY) =
        -wz * dt_s;  // Corrected sign

    return state_jacobian;
}

}  // namespace estimation
