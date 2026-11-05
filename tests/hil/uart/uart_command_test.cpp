#include "DrivetrainUARTCommand.hpp"
#include "UARTComms.hpp"
#include <iostream>
#include <limits>

int main() {
    try {
        auto require = [](bool ok) { if (!ok) throw std::runtime_error("Command validation failed"); };
        const auto command = comms::make_drivetrain_uart_command({1, -2, 3, -4}, {0, 2, 3, 4}, 42);
        require(command.desired_rpms().fr() == -2 && command.desired_rpms().rr() == -4);
        require(command.torque_limit_nm().fl() == 0 && command.torque_limit_nm().rl() == 3);
        require(command.prev_mcu_recv_millis() == 42);
        for (unsigned wheel = 0; wheel < 4; ++wheel) {
            for (float bad : {-1.f, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::quiet_NaN()}) {
                std::array<float, 4> torque{}; torque[wheel] = bad;
                bool rejected = false;
                try { comms::make_drivetrain_uart_command({}, torque); } catch (const std::invalid_argument&) { rejected = true; }
                require(rejected);
            }
            std::array<float, 4> rpm{}; rpm[wheel] = std::numeric_limits<float>::quiet_NaN();
            bool rejected = false;
            try { comms::make_drivetrain_uart_command(rpm, {}); } catch (const std::invalid_argument&) { rejected = true; }
            require(rejected);
        }
        comms::UARTComms::Config config; config.device_name = "/unused";
        hytech_msgs::MCUOutputData feedback;
        comms::UARTComms uart(config, feedback, [](auto) {});
        require(!uart.send(command));
        uart.stop(); uart.stop();
        for (unsigned i = 0; i < 3; ++i) {
            bool failed = false;
            try { uart.start(); } catch (const std::exception&) { failed = true; }
            require(failed);
            uart.stop(); uart.stop();
            require(!uart.is_running());
            require(!uart.send(command));
        }
        std::cout << "PASS validation and stopped sends\n";
        return 0;
    } catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
}
