#include <gtest/gtest.h>

#include <GssModel.hpp>

namespace estimation
{
namespace
{

constexpr double kTolerance = 1e-9;

TEST(GssModelTest, PredictsSensorVelocityFromFluYawRateAndOffset)
{
    InternalEstimatorState state;
    state.vx_body_mps() = 10.0;
    state.vy_body_mps() = 2.0;

    const double yaw_rate_radps = 3.0;
    const transforms::Pose2D T_base_gss{1.0, 0.25, 0.0};

    const GssMeasurementEigen predicted =
        predict_gss_measurement(state, yaw_rate_radps, T_base_gss);

    EXPECT_NEAR(predicted.vx_sensor_mps(), 9.25, kTolerance);
    EXPECT_NEAR(predicted.vy_sensor_mps(), 5.0, kTolerance);
}

TEST(GssModelTest, JacobianDependsOnlyOnBodyVelocityStates)
{
    const transforms::Pose2D T_base_gss{1.0, 0.25, 0.0};

    const GssMeasurementJacobian jacobian =
        compute_gss_measurement_jacobian(T_base_gss);

    EXPECT_NEAR(jacobian(GssMeasurementIndex::VX_SENSOR, StateIndex::VX_BODY),
                1.0, kTolerance);
    EXPECT_NEAR(jacobian(GssMeasurementIndex::VY_SENSOR, StateIndex::VY_BODY),
                1.0, kTolerance);
    EXPECT_NEAR(jacobian(GssMeasurementIndex::VX_SENSOR, StateIndex::VY_BODY),
                0.0, kTolerance);
    EXPECT_NEAR(jacobian(GssMeasurementIndex::VY_SENSOR, StateIndex::VX_BODY),
                0.0, kTolerance);
}

}  // namespace
}  // namespace estimation
