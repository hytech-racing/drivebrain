
#pragma once

#include <cstdint>

#include "EstimatorMeasurements.hpp"
#include "EstimatorTypes.hpp"
#include "StateEstimate.hpp"

namespace estimation
{

class EkfEstimator
{
   public:
    explicit EkfEstimator(const EkfParams& params,
                          const GssSensorConfig& gss_sensor_config,
                          const transforms::Pose2D& T_base_gss);

    void initialize(const InternalEstimatorState& initial_state,
                    const StateCovariance& initial_covariance);

    void predict(const ImuMeasurement& input, double dt_s);

    void update_gss_speed(const GssMeasurementEigen& measurement);

    [[nodiscard]] const InternalEstimatorState& state() const;
    [[nodiscard]] const StateCovariance& covariance() const;
    [[nodiscard]] bool initialized() const;
    [[nodiscard]] std::uint64_t filter_timestamp_ns() const;

    StateEstimate state_estimate() const;

   private:
    void _update_generic(const Eigen::VectorXd& residual,
                         const Eigen::MatrixXd& H, const Eigen::MatrixXd& R);

    void _wrap_state_angles();

    void _enforce_covariance_safety();

   private:
    EkfParams _params;
    GssSensorConfig _gss_sensor_config{};

    transforms::Pose2D _T_base_gss{};

    std::uint64_t _filter_timestamp_ns{};
    double _latest_yaw_rate_vehicle_flu_radps{};

    InternalEstimatorState _state;
    StateCovariance _covariance;
    bool _initialized{false};
};

}  // namespace estimation
