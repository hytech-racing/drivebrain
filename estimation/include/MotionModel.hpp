#pragma once

#include "EstimatorMeasurements.hpp"
#include "EstimatorTypes.hpp"

namespace estimation
{

[[nodiscard]] InternalEstimatorState predict_nominal_state(
    const InternalEstimatorState& previous_state, const ImuMeasurement& input,
    const double dt_s);

[[nodiscard]] StateJacobian compute_state_transition_jacobian(
    const InternalEstimatorState& previous_state, const ImuMeasurement& input,
    const double dt_s);
}  // namespace estimation
