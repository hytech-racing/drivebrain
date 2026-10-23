#include "OdomEkf.h"

#include <cmath>
#include <string>

#include <FoxgloveServer.hpp>

using namespace estimation;

namespace {

enum Index { X, Y, YAW, VX, VY };

}  // namespace

OdomEkf::OdomEkf() {
    _position_process_std_m_per_sqrt_s =
        core::FoxgloveServer::instance().get_param<float>("Odometry/position_process_std_m_per_sqrt_s").value();
    _yaw_process_std_rad_per_sqrt_s =
        core::FoxgloveServer::instance().get_param<float>("Odometry/yaw_process_std_rad_per_sqrt_s").value();
    _velocity_process_std_mps_per_sqrt_s =
        core::FoxgloveServer::instance().get_param<float>("Odometry/velocity_process_std_mps_per_sqrt_s").value();
    _gss_vx_noise_std_mps = core::FoxgloveServer::instance().get_param<float>("Odometry/gss_vx_noise_std_mps").value();
    _gss_vy_noise_std_mps = core::FoxgloveServer::instance().get_param<float>("Odometry/gss_vy_noise_std_mps").value();
    _gss_position_base_m = {core::FoxgloveServer::instance().get_param<float>("Odometry/gss_position_base_x_m").value(),
                            core::FoxgloveServer::instance().get_param<float>("Odometry/gss_position_base_y_m").value()};

    const double position_std = core::FoxgloveServer::instance().get_param<float>("Odometry/initial_position_std_m").value();
    const double yaw_std = core::FoxgloveServer::instance().get_param<float>("Odometry/initial_yaw_std_rad").value();
    const double velocity_std = core::FoxgloveServer::instance().get_param<float>("Odometry/initial_velocity_std_mps").value();
    _P.diagonal() = (State() << position_std, position_std, yaw_std, velocity_std, velocity_std)
                        .finished().cwiseAbs2();
    _wrap_yaw_and_bound_covariance();
}

bool OdomEkf::update(const ImuMeasurement& imu) {
    if (!_accept(imu.timestamp_ns, _latest_imu_ns)) {
        return false;
    }

    if (!_previous_imu_ns) {
        _previous_imu_ns = imu.timestamp_ns;
        return false;
    }

    _predict(imu, static_cast<double>(imu.timestamp_ns - *_previous_imu_ns) / 1e9);
    _previous_imu_ns = imu.timestamp_ns;
    _timestamp_ns = imu.timestamp_ns;
    _yaw_rate_radps = imu.yaw_rate_radps;
    return true;
}

bool OdomEkf::update(const GssMeasurement& gss) {
    if (!_accept(gss.timestamp_ns, _latest_gss_ns)) {
        return false;
    }

    // Body velocity seen at the sensor's lever arm
    const Eigen::Vector2d& lever_m = _gss_position_base_m;
    const Eigen::Vector2d predicted{_x(VX) - _yaw_rate_radps * lever_m.y(),
                                    _x(VY) + _yaw_rate_radps * lever_m.x()};
    const Eigen::Vector2d residual = Eigen::Vector2d{gss.vx_mps, gss.vy_mps} - predicted;

    Eigen::Matrix<double, 2, 5> H = Eigen::Matrix<double, 2, 5>::Zero();
    H(0, VX) = 1.0;
    H(1, VY) = 1.0;

    const Eigen::Matrix2d R =
        Eigen::Vector2d{_gss_vx_noise_std_mps, _gss_vy_noise_std_mps}.cwiseAbs2().asDiagonal();
    const Eigen::Matrix2d S = H * _P * H.transpose() + R;
    const Eigen::Matrix<double, 5, 2> K = _P * H.transpose() * S.inverse();

    _x += K * residual;

    // Joseph form
    const Covariance I_KH = Covariance::Identity() - K * H;
    _P = I_KH * _P * I_KH.transpose() + K * R * K.transpose();
    _wrap_yaw_and_bound_covariance();
    return true;
}

Eigen::Isometry2d OdomEkf::pose_odom_from_base() const {
    return core::geometry::make_pose2d(_x(X), _x(Y), _x(YAW));
}

Eigen::Vector2d OdomEkf::velocity_base_mps() const { return {_x(VX), _x(VY)}; }

bool OdomEkf::_accept(const std::uint64_t timestamp_ns, std::uint64_t& latest_sensor_ns) {
    if (timestamp_ns < latest_sensor_ns) {
        return false;
    }
    latest_sensor_ns = timestamp_ns;
    return _timestamp_ns == 0 || timestamp_ns >= _timestamp_ns;
}

void OdomEkf::_predict(const ImuMeasurement& imu, const double dt) {
    const double c = std::cos(_x(YAW));
    const double s = std::sin(_x(YAW));
    const double vx = _x(VX);
    const double vy = _x(VY);
    const double wz = imu.yaw_rate_radps;
    const double dt2 = dt * dt;

    // Body-frame velocity derivative, including the rotating-frame terms
    const double ax = imu.ax_mps2 + vy * wz;
    const double ay = imu.ay_mps2 - vx * wz;

    Covariance F = Covariance::Identity();
    F(X, YAW) = dt * (-vx * s - vy * c) + 0.5 * dt2 * (-ax * s - ay * c);
    F(X, VX) = dt * c + 0.5 * dt2 * wz * s;
    F(X, VY) = -dt * s + 0.5 * dt2 * wz * c;
    F(Y, YAW) = dt * (vx * c - vy * s) + 0.5 * dt2 * (ax * c - ay * s);
    F(Y, VX) = dt * s - 0.5 * dt2 * wz * c;
    F(Y, VY) = dt * c + 0.5 * dt2 * wz * s;
    F(VX, VY) = wz * dt;
    F(VY, VX) = -wz * dt;

    _x(X) += dt * (vx * c - vy * s) + 0.5 * ax * dt2 * c - 0.5 * ay * dt2 * s;
    _x(Y) += dt * (vx * s + vy * c) + 0.5 * ax * dt2 * s + 0.5 * ay * dt2 * c;
    _x(YAW) += dt * wz;
    _x(VX) += ax * dt;
    _x(VY) += ay * dt;

    Covariance Q = Covariance::Zero();
    Q.diagonal() = (State() << _position_process_std_m_per_sqrt_s, _position_process_std_m_per_sqrt_s,
                    _yaw_process_std_rad_per_sqrt_s, _velocity_process_std_mps_per_sqrt_s,
                    _velocity_process_std_mps_per_sqrt_s).finished().cwiseAbs2();

    _P = F * _P * F.transpose() + Q * dt;
    _wrap_yaw_and_bound_covariance();
}

void OdomEkf::_wrap_yaw_and_bound_covariance() {
    _x(YAW) = std::remainder(_x(YAW), 2.0 * M_PI);
    _P = 0.5 * (_P + _P.transpose());
    _P.diagonal() = _P.diagonal().cwiseMax(1e-9).cwiseMin(1e6);
}
