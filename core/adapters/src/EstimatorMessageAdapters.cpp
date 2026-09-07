#include "EstimatorMessageAdapters.hpp"

#include <cmath>
#include <cstdint>
#include <optional>

namespace adapters
{

std::optional<estimation::ImuMeasurement> to_imu_measurement(
    const hytech_msgs::VnImuData& message)
{
    const double ax = message.comp_no_gravity_accel_vehicle_flu_m_ss().x();
    const double ay = message.comp_no_gravity_accel_vehicle_flu_m_ss().y();
    const double gz = message.comp_gyro_vehicle_flu_rad_s().z();
    const std::uint64_t stamp = message.vn_time_startup_ns();

    if (!std::isfinite(ax) || !std::isfinite(ay) || !std::isfinite(gz) ||
        stamp == 0)
    {
        return std::nullopt;
    }

    return estimation::ImuMeasurement{stamp, ax, ay, gz};
}

std::optional<estimation::GssMeasurement> to_gss_measurement(
    const hytech_msgs::GssData& message)
{
    const double vx = message.vel_sensor_flu_mps().x();
    const double vy = message.vel_sensor_flu_mps().y();
    const std::uint64_t stamp = message.sensor_timestamp_ns();

    if (!std::isfinite(vx) || !std::isfinite(vy) || stamp == 0)
    {
        return std::nullopt;
    }

    return estimation::GssMeasurement{stamp, vx, vy};
}

}  // namespace adapters
