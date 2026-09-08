#pragma once

#include <cstdint>
#include <variant>

#include "EstimatorTypes.hpp"

namespace estimation
{

struct ImuMeasurement
{
    std::uint64_t timestamp_ns{};

    // gravity-removed accel at IMU sensor location, expressed in vehicle FLU
    double ax_vehicle_flu_mps2{};
    double ay_vehicle_flu_mps2{};

    double yaw_rate_vehicle_flu_radps{};
};

struct GssMeasurement
{
    std::uint64_t timestamp_ns{};

    // velocity at the GSS sensor location, expressed in vehicle-aligned FLU
    double vx_sensor_flu_mps{};
    double vy_sensor_flu_mps{};
};

struct GssMeasurementEigen
{
    GssMeasurementVector z = GssMeasurementVector::Zero();

    std::uint64_t timestamp_ns{};

    [[nodiscard]] double vx_sensor_mps() const noexcept
    {
        return z(GssMeasurementIndex::VX_SENSOR);
    }

    [[nodiscard]] double vy_sensor_mps() const noexcept
    {
        return z(GssMeasurementIndex::VY_SENSOR);
    }

    double& vx_sensor_mps() noexcept
    {
        return z(GssMeasurementIndex::VX_SENSOR);
    }

    double& vy_sensor_mps() noexcept
    {
        return z(GssMeasurementIndex::VY_SENSOR);
    }
};

using EstimatorEvent = std::variant<ImuMeasurement, GssMeasurement>;

}  // namespace estimation
