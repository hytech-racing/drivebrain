# UART command contract

This target compiles the real `UARTComms.cpp` without the full Drivebrain app.
It checks command construction, the wire codec against an independent Python
implementation, and real asynchronous RX/TX through a pseudo-terminal. It is a
software check, not a hardware timing test or the planned stress-test runner.

## Build

On Raspberry Pi OS 64-bit install `build-essential cmake libboost-dev
libprotobuf-dev protobuf-compiler libspdlog-dev python3`. On macOS, Homebrew
`cmake boost protobuf spdlog` provides the corresponding packages.

From the Drivebrain root:

```sh
cmake -S tests/uart -B build-uart-contract -DHT_PROTO_DIR=/absolute/path/to/HT_proto/proto
cmake --build build-uart-contract -j4
ctest --test-dir build-uart-contract --output-on-failure
```

Use the same pinned schemas at both endpoints. The current VCR release uses the
same MCUCommandData/vector field numbers and types as the sibling HT_proto
sources. Exported test fixtures are independently checked for wire compatibility.
No production `.proto` change is needed.

## Commands

```cpp
#include "DrivetrainUARTCommand.hpp"
#include "UARTComms.hpp"

auto command = comms::make_drivetrain_uart_command(
    {100.f, 200.f, 300.f, 400.f}, // FL, FR, RL, RR; RPM
    {1.f, 2.f, 3.f, 4.f});      // Torque allowances in N*m
comms::UARTComms::Config config;
config.device_name = "/dev/ttyAMA0"; // Verify GPIO mapping and disable serial console.
config.baud_rate = 921600;
// Choose the receive prototype to match the peer's actual outgoing message type.
hytech_msgs::MCUOutputData feedback;
comms::UARTComms uart(config, feedback, [](auto message) { /* Handle feedback. */ });
uart.start();
bool accepted = uart.send(command);
auto counters = uart.stats();
uart.stop();
```

`send()` means queue acceptance, not remote receipt. Stats are cumulative since
construction; snapshots can observe counters advancing independently. `rx_frames`
counts successfully parsed Protobufs before callbacks. This generic driver does
not validate vehicle commands: the helper and VCR decoder enforce that contract.
Signed RPMs are representable; allowances must be finite nonnegative magnitudes.
Convert signed controller torque limits deliberately before using this helper.
The helper preserves the legacy timestamp and does not repurpose it as a sequence.

Logging now defaults to disabled. Application users who need MCAP/Foxglove should
initialize those services and set `config.logger = [](auto message, const auto&
topic) { core::log(message, topic); };` (with Telemetry.hpp included by the caller).
Logger and receive callbacks run on the UART worker. They must not block or call
start/stop. Exceptions are counted and do not terminate reception.

Wire format is **COBS(payload || CRC16 little-endian) || 0x00**, without an opening
delimiter or message-kind byte. CRC-16/CCITT-FALSE covers only serialized payload:
polynomial 0x1021, init 0xffff, no reflection, xorout 0, check `123456789` = 0x29b1.
Empty delimiters are ignored. Invalid frames are rejected and counted.

The transport has no type handshake, application ACK, replay detection, heartbeat,
or automatic reconnect. A stopped or disconnected application must explicitly
stop/restart the driver. Production freshness and source switching are not wired
by this change. Do not infer motor actuation or a healthy peer from `is_running()`.
