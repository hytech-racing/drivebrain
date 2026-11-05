#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include "hytech_msgs.pb.h"

namespace comms {
// Wheel order is FL, FR, RL, RR. Limits are nonnegative magnitudes in N*m;
// RPM setpoints may be signed. Vehicle-specific physical bounds belong to the controller.
inline hytech_msgs::MCUCommandData make_drivetrain_uart_command(
    const std::array<float, 4>& desired_rpms,
    const std::array<float, 4>& torque_limits_nm,
    int64_t previous_vcr_receive_millis = 0) {
    for (std::size_t i = 0; i < 4; ++i) {
        if (!std::isfinite(desired_rpms[i]) || !std::isfinite(torque_limits_nm[i]) || torque_limits_nm[i] < 0) {
            throw std::invalid_argument("UART commands require finite RPMs and finite nonnegative torque limits");
        }
    }
    hytech_msgs::MCUCommandData command;
    auto assign = [](hytech_msgs::veh_vec_float* out, const std::array<float, 4>& values) {
        out->set_fl(values[0]); out->set_fr(values[1]); out->set_rl(values[2]); out->set_rr(values[3]);
    };
    assign(command.mutable_desired_rpms(), desired_rpms);
    assign(command.mutable_torque_limit_nm(), torque_limits_nm);
    command.set_prev_mcu_recv_millis(previous_vcr_receive_millis);
    return command;
}
}
