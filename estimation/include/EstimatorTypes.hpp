
#pragma once
#include <Eigen/Dense>

#include "RigidTransform2D.hpp"

namespace estimation
{

struct StateIndex
{
    static constexpr Eigen::Index X_ODOM = 0;
    static constexpr Eigen::Index Y_ODOM = 1;
    static constexpr Eigen::Index YAW_ODOM = 2;
    static constexpr Eigen::Index VX_BODY = 3;
    static constexpr Eigen::Index VY_BODY = 4;

    static constexpr int SIZE = 5;
};

struct GssMeasurementIndex
{
    static constexpr Eigen::Index VX_SENSOR = 0;
    static constexpr Eigen::Index VY_SENSOR = 1;
    static constexpr Eigen::Index SIZE = 2;
};

struct ImuMeasurementIndex
{
    static constexpr Eigen::Index AX_SENSOR = 0;
    static constexpr Eigen::Index AY_SENSOR = 1;
    static constexpr Eigen::Index WZ_SENSOR = 2;
    static constexpr Eigen::Index SIZE = 3;
};

using StateVector = Eigen::Matrix<double, StateIndex::SIZE, 1>;
using StateCovariance =
    Eigen::Matrix<double, StateIndex::SIZE, StateIndex::SIZE>;
using StateJacobian = Eigen::Matrix<double, StateIndex::SIZE, StateIndex::SIZE>;

using ImuMeasurementVector =
    Eigen::Matrix<double, ImuMeasurementIndex::SIZE, 1>;
using ImuMeasurementCovariance =
    Eigen::Matrix<double, ImuMeasurementIndex::SIZE, ImuMeasurementIndex::SIZE>;

using GssMeasurementVector =
    Eigen::Matrix<double, GssMeasurementIndex::SIZE, 1>;
using GssMeasurementCovariance =
    Eigen::Matrix<double, GssMeasurementIndex::SIZE, GssMeasurementIndex::SIZE>;
using GssMeasurementJacobian =
    Eigen::Matrix<double, GssMeasurementIndex::SIZE, StateIndex::SIZE>;

struct InternalEstimatorState
{
    StateVector x = StateVector::Zero();

    [[nodiscard]] double x_odom_m() const noexcept
    {
        return x(StateIndex::X_ODOM);
    }
    [[nodiscard]] double y_odom_m() const noexcept
    {
        return x(StateIndex::Y_ODOM);
    }
    [[nodiscard]] double yaw_odom_rad() const noexcept
    {
        return x(StateIndex::YAW_ODOM);
    }
    [[nodiscard]] double vx_body_mps() const noexcept
    {
        return x(StateIndex::VX_BODY);
    }
    [[nodiscard]] double vy_body_mps() const noexcept
    {
        return x(StateIndex::VY_BODY);
    }
    double& x_odom_m() noexcept { return x(StateIndex::X_ODOM); }
    double& y_odom_m() noexcept { return x(StateIndex::Y_ODOM); }
    double& yaw_odom_rad() noexcept { return x(StateIndex::YAW_ODOM); }
    double& vx_body_mps() noexcept { return x(StateIndex::VX_BODY); }
    double& vy_body_mps() noexcept { return x(StateIndex::VY_BODY); }
};

struct GssSensorConfig
{
    double vx_noise_std_mps{};
    double vy_noise_std_mps{};
};

struct EkfParams
{
    double initial_position_std_m{};
    double initial_yaw_std_rad{};
    double initial_velocity_std_mps{};

    // Continuous time noise densities, needs to be multiplied by dt
    double position_process_std_m_per_sqrt_s{};
    double yaw_process_std_rad_per_sqrt_s{};
    double velocity_process_std_mps_per_sqrt_s{};
};

}  // namespace estimation
