#include "EkfEstimator.hpp"

#include <cmath>

#include "GssModel.hpp"
#include "MotionModel.hpp"

namespace estimation
{

namespace
{

constexpr double kPi = 3.14159265358979323846;

double wrap_angle(double angle_rad)
{
    while (angle_rad > kPi)
    {
        angle_rad -= 2.0 * kPi;
    }

    while (angle_rad < -kPi)
    {
        angle_rad += 2.0 * kPi;
    }

    return angle_rad;
}
}  // namespace

EkfEstimator::EkfEstimator(const EkfParams& params,
                           const GssSensorConfig& optical_sensor_config,
                           const transforms::Pose2D& T_base_gss)
    : _params(params),
      _gss_sensor_config(optical_sensor_config),
      _T_base_gss(T_base_gss)
{
}

void EkfEstimator::initialize(const InternalEstimatorState& initial_state,
                              const StateCovariance& initial_covariance)
{
    _state = initial_state;
    _covariance = initial_covariance;
    _initialized = true;

    _wrap_state_angles();
    _enforce_covariance_safety();
}

void EkfEstimator::predict(const ImuMeasurement& input, double dt_s)
{
    if (!_initialized)
    {
        return;
    }

    InternalEstimatorState previous_state = _state;
    StateJacobian F =
        compute_state_transition_jacobian(previous_state, input, dt_s);

    _state = predict_nominal_state(previous_state, input, dt_s);

    StateCovariance Qd = StateCovariance::Zero();

    Qd(StateIndex::X_ODOM, StateIndex::X_ODOM) =
        _params.position_process_std_m_per_sqrt_s *
        _params.position_process_std_m_per_sqrt_s * dt_s;
    Qd(StateIndex::Y_ODOM, StateIndex::Y_ODOM) =
        _params.position_process_std_m_per_sqrt_s *
        _params.position_process_std_m_per_sqrt_s * dt_s;

    Qd(StateIndex::YAW_ODOM, StateIndex::YAW_ODOM) =
        _params.yaw_process_std_rad_per_sqrt_s *
        _params.yaw_process_std_rad_per_sqrt_s * dt_s;

    Qd(StateIndex::VX_BODY, StateIndex::VX_BODY) =
        _params.velocity_process_std_mps_per_sqrt_s *
        _params.velocity_process_std_mps_per_sqrt_s * dt_s;
    Qd(StateIndex::VY_BODY, StateIndex::VY_BODY) =
        _params.velocity_process_std_mps_per_sqrt_s *
        _params.velocity_process_std_mps_per_sqrt_s * dt_s;

    _covariance = F * _covariance * F.transpose() + Qd;

    _wrap_state_angles();
    _enforce_covariance_safety();

    _filter_timestamp_ns = input.timestamp_ns;
    _latest_yaw_rate_vehicle_flu_radps = input.yaw_rate_vehicle_flu_radps;
}

void EkfEstimator::update_gss_speed(const GssMeasurementEigen& measurement)
{
    if (!_initialized)
    {
        return;
    }

    const GssMeasurementEigen predicted_measurement = predict_gss_measurement(
        _state, _latest_yaw_rate_vehicle_flu_radps, _T_base_gss);

    const GssMeasurementVector residual =
        measurement.z - predicted_measurement.z;

    const GssMeasurementJacobian H =
        compute_gss_measurement_jacobian(_T_base_gss);

    Eigen::Matrix<double, GssMeasurementIndex::SIZE, GssMeasurementIndex::SIZE>
        R;
    R.setIdentity();
    R(GssMeasurementIndex::VX_SENSOR, GssMeasurementIndex::VX_SENSOR) =
        _gss_sensor_config.vx_noise_std_mps *
        _gss_sensor_config.vx_noise_std_mps;
    R(GssMeasurementIndex::VY_SENSOR, GssMeasurementIndex::VY_SENSOR) =
        _gss_sensor_config.vy_noise_std_mps *
        _gss_sensor_config.vy_noise_std_mps;

    _update_generic(residual, H, R);

    _filter_timestamp_ns = measurement.timestamp_ns;
}

const InternalEstimatorState& EkfEstimator::state() const { return _state; }

const StateCovariance& EkfEstimator::covariance() const { return _covariance; }

bool EkfEstimator::initialized() const { return _initialized; }

std::uint64_t EkfEstimator::filter_timestamp_ns() const
{
    return _filter_timestamp_ns;
}

void EkfEstimator::_update_generic(const Eigen::VectorXd& residual,
                                   const Eigen::MatrixXd& H,
                                   const Eigen::MatrixXd& R)
{
    if (!_initialized)
    {
        return;
    }

    // innovation covariance
    Eigen::MatrixXd S = H * _covariance * H.transpose() + R;

    // kalman gain
    Eigen::MatrixXd K = _covariance * H.transpose() * S.inverse();

    // correction
    Eigen::VectorXd correction = K * residual;

    _state.x = _state.x + correction;

    // wrap yaw
    _wrap_state_angles();

    // Joseph form covariance update
    Eigen::Matrix<double, StateIndex::SIZE, StateIndex::SIZE> I;
    I.setIdentity();
    _covariance = (I - K * H) * _covariance * (I - K * H).transpose() +
                  K * R * K.transpose();

    // check covariance
    _enforce_covariance_safety();
}

void EkfEstimator::_wrap_state_angles()
{
    _state.yaw_odom_rad() = wrap_angle(_state.yaw_odom_rad());
}

void EkfEstimator::_enforce_covariance_safety()
{
    // enforce symmetry
    _covariance = 0.5 * (_covariance + _covariance.transpose());

    // diagonal entires should be >= 0
    _covariance.diagonal() = _covariance.diagonal().cwiseMax(1e-9);

    // restrict diagonal entries from growing infinitely
    _covariance.diagonal() = _covariance.diagonal().cwiseMin(1e6);
}

StateEstimate EkfEstimator::state_estimate() const
{
    StateEstimate estimate;

    estimate.timestamp_ns = _filter_timestamp_ns;
    estimate.initialized = initialized();
    estimate.x_odom_m = _state.x_odom_m();
    estimate.y_odom_m = _state.y_odom_m();
    estimate.yaw_odom_rad = _state.yaw_odom_rad();
    estimate.vx_vehicle_flu_mps = _state.vx_body_mps();
    estimate.vy_vehicle_flu_mps = _state.vy_body_mps();
    estimate.yaw_rate_vehicle_flu_radps = _latest_yaw_rate_vehicle_flu_radps;

    return estimate;
}

}  // namespace estimation
