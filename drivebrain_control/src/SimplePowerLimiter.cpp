#include "SimplePowerLimiter.hpp"

#include <cmath>

namespace control {

    core::SpeedControlOut apply_power_limit(core::SpeedControlOut current_control,
                                             veh_vec<float> current_rpms, float max_power_kw) {
    auto cmd_out = current_control;
    // Apply power limit (basically a re-implementation of MCU)
    float net_torque_mag = 0;
    float net_power = 0;

    net_torque_mag += ::fabs(cmd_out.torque_lim_nm.FL);
    net_torque_mag += ::fabs(cmd_out.torque_lim_nm.FR);
    net_torque_mag += ::fabs(cmd_out.torque_lim_nm.RL);
    net_torque_mag += ::fabs(cmd_out.torque_lim_nm.RR);

    net_power +=
        ::fabs(cmd_out.torque_lim_nm.FL) * (current_rpms.FL * constants::RPM_TO_RAD_PER_SECOND);
    net_power +=
        ::fabs(cmd_out.torque_lim_nm.FR) * (current_rpms.FR * constants::RPM_TO_RAD_PER_SECOND);
    net_power +=
        ::fabs(cmd_out.torque_lim_nm.RL) * (current_rpms.RL * constants::RPM_TO_RAD_PER_SECOND);
    net_power +=
        ::fabs(cmd_out.torque_lim_nm.RR) * (current_rpms.RR * constants::RPM_TO_RAD_PER_SECOND);

    auto max_power_watt = max_power_kw * 1000.0f;
    if (net_power > max_power_watt) {
        /* FL */
        // 1. Calculate the torque percent (individual torque/total torque)
        // 2. Multiply the torque percent by the power limit to ensure that all four powers add up
        // to power limit
        float torque_percent_FL = ::fabs(cmd_out.torque_lim_nm.FL / net_torque_mag);
        float power_per_corner_FL = (torque_percent_FL * max_power_watt);

        // 3. Divide power by rads per seconds to get torque per corner
        cmd_out.torque_lim_nm.FL =
            ::fabs(power_per_corner_FL / (current_rpms.FL * constants::RPM_TO_RAD_PER_SECOND));

        /* FR */
        // 1. Calculate the torque percent (individual torque/total torque)
        // 2. Multiply the torque percent by the power limit to ensure that all four powers add up
        // to power limit
        float torque_percent_FR = ::fabs(cmd_out.torque_lim_nm.FR / net_torque_mag);
        float power_per_corner_FR = (torque_percent_FR * max_power_watt);

        // 3. Divide power by rads per seconds to get torque per corner
        cmd_out.torque_lim_nm.FR =
            ::fabs(power_per_corner_FR / (current_rpms.FR * constants::RPM_TO_RAD_PER_SECOND));

        /* RL */
        // 1. Calculate the torque percent (individual torque/total torque)
        // 2. Multiply the torque percent by the power limit to ensure that all four powers add up
        // to power limit
        float torque_percent_RL = ::fabs(cmd_out.torque_lim_nm.RL / net_torque_mag);
        float power_per_corner_RL = (torque_percent_RL * max_power_watt);

        // 3. Divide power by rads per seconds to get torque per corner
        cmd_out.torque_lim_nm.RL =
            ::fabs(power_per_corner_RL / (current_rpms.RL * constants::RPM_TO_RAD_PER_SECOND));

        /* RR */
        // 1. Calculate the torque percent (individual torque/total torque)
        // 2. Multiply the torque percent by the power limit to ensure that all four powers add up
        // to power limit
        float torque_percent_RR = ::fabs(cmd_out.torque_lim_nm.RR / net_torque_mag);
        float power_per_corner_RR = (torque_percent_RR * max_power_watt);

        // 3. Divide power by rads per seconds to get torque per corner
        cmd_out.torque_lim_nm.RR =
            ::fabs(power_per_corner_RR / (current_rpms.RR * constants::RPM_TO_RAD_PER_SECOND));
    }

    // std::cout <<"cmd_out.torque_lim_nm.FL " << cmd_out.torque_lim_nm.FL <<std::endl;
    return cmd_out;
}
} // namespace control