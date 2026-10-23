#pragma once

#include <cstdint>
#include <optional>

#include <Eigen/Dense>

#include <Geometry.hpp>

namespace estimation {

/**
 * Gravity-removed acceleration at the IMU, expressed in vehicle FLU
 */
struct ImuMeasurement {
    std::uint64_t timestamp_ns{};
    double ax_mps2{};
    double ay_mps2{};
    double yaw_rate_radps{};
};

/**
 * Velocity at the GSS sensor location, expressed in vehicle-aligned FLU
 */
struct GssMeasurement {
    std::uint64_t timestamp_ns{};
    double vx_mps{};
    double vy_mps{};
};

/**
 * Planar odometry EKF. State is [x, y, yaw] in odom plus [vx, vy] in base_link,
 * predicted with IMU acceleration and yaw rate and corrected with GSS velocity.
 * A measurement older than the previous one from the same sensor, or older
 * than the filter, is rejected.
 */
class OdomEkf {
public:
    /* Reads the noise model and GSS position from the Odometry block of the config */
    OdomEkf();

    /**
     * Predicts to the measurement time. The first IMU sample only starts the clock.
     *
     * @return whether the estimate changed
     */
    bool update(const ImuMeasurement& imu);

    /**
     * Corrects body velocity, using the latest IMU yaw rate for the lever arm.
     *
     * @return whether the estimate changed
     */
    bool update(const GssMeasurement& gss);

    std::uint64_t timestamp_ns() const { return _timestamp_ns; }
    Eigen::Isometry2d pose_odom_from_base() const;
    Eigen::Vector2d velocity_base_mps() const;
    double yaw_rate_radps() const { return _yaw_rate_radps; }

private:
    using State = Eigen::Matrix<double, 5, 1>;
    using Covariance = Eigen::Matrix<double, 5, 5>;

    bool _accept(std::uint64_t timestamp_ns, std::uint64_t& latest_sensor_ns);
    void _predict(const ImuMeasurement& imu, double dt_s);
    void _wrap_yaw_and_bound_covariance();

    // Continuous-time noise densities, scaled by dt
    double _position_process_std_m_per_sqrt_s{};
    double _yaw_process_std_rad_per_sqrt_s{};
    double _velocity_process_std_mps_per_sqrt_s{};
    double _gss_vx_noise_std_mps{};
    double _gss_vy_noise_std_mps{};
    // GSS mounting position in base_link
    Eigen::Vector2d _gss_position_base_m = Eigen::Vector2d::Zero();

    State _x = State::Zero();
    Covariance _P = Covariance::Zero();
    std::uint64_t _timestamp_ns{};
    double _yaw_rate_radps{};
    std::uint64_t _latest_imu_ns{};
    std::uint64_t _latest_gss_ns{};
    std::optional<std::uint64_t> _previous_imu_ns;
};

}  // namespace estimation
