#pragma once

#include <cstdint>

namespace estimation
{

// Public facing named state estimates
struct StateEstimate
{
    std::uint64_t timestamp_ns{};
    std::uint64_t sequence{};

    double x_odom_m{};
    double y_odom_m{};
    double yaw_odom_rad{};

    double vx_vehicle_flu_mps{};
    double vy_vehicle_flu_mps{};

    double yaw_rate_vehicle_flu_radps{};

    bool initialized{};
};

}  // namespace estimation
