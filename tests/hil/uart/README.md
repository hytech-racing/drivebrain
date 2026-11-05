# UART command bench for abdur/uart-driver

This builds the actual UARTComms.cpp from this branch independently of cameras,
CAN, Foxglove and MCAP. Commands contain four RPM setpoints and four nonnegative
torque limits in N*m, using the existing hytech_msgs.MCUCommandData schema.
Wheel order is FL, FR, RL, RR. RPMs may be signed; torque limits are magnitudes.
The helper rejects nonfinite values and negative limits. Vehicle-specific bounds
and validation on the VCR remain necessary.

This is bench tooling, with no production control routing or VCR firmware changes.
A decoded reply proves communication, not motor actuation.

## Build

Install build-essential, cmake, libboost-dev, libssl-dev, libspdlog-dev,
libprotobuf-dev, protobuf-compiler and python3 on Raspberry Pi OS Lite 64-bit.
Use the same pinned HT_proto source on both endpoints. From this worktree root:

```sh
cmake -S tests/hil/uart -B build-uart -DHT_PROTO_DIR=/absolute/path/to/HT_proto/proto
cmake --build build-uart -j4
ctest --test-dir build-uart --output-on-failure
```

On macOS install the equivalent Homebrew packages and supply
`-DCMAKE_PREFIX_PATH=/opt/homebrew -DOPENSSL_ROOT_DIR=/opt/homebrew/opt/openssl@3`
if needed. The production project keeps its existing build dependencies.

## Run

After configuring a verified, unused hardware UART, run:

```sh
./build-uart/uart_hil_test --device /dev/serial0 --role pi --count 1000 \
  --rpms 100,200,-300,400 --torque-limits 1,2,3,4
```

Verify the actual device node for your Pi model. Defaults: 115200 8N1 without flow
control, 512-byte payload limit, 256 pattern bytes, five-second receive/transmit
drain deadlines, and a 10 ms pause after each successful exchange. This pause is
not a fixed-period control scheduler. Run --help for other options.

A second Linux/macOS endpoint can use --role peer on its connected UART with the
same --count. On an actual Teensy, implement the peer contract below first.
The peer decodes and reconstructs each reply; the Pi validates run ID, sequence,
all setpoints/limits and every pattern byte. RTT uses one host's monotonic clock.

The test-only uart_bench.Command wrapper carries MCUCommandData, run ID, sequence
and bounded bytes. uart_bench.Reply carries a decoded Command. This lets tests
correlate replies without repurposing production fields such as prev_MCU_recv_millis.
The production command and test wrapper are different types; configure the chosen
receive prototype accordingly on each endpoint.

CTest connects two PTYs and runs the real driver. It covers 1,000 exchanges,
zero/default fields, binary data and COBS boundaries, fragmented input,
corrupt CRC, malformed COBS, CRC-valid invalid Protobuf, logger exceptions,
local/remote payload limits, reply loss, command validation and failed-start
cleanup. Every emitted frame is checked with an independent Python COBS decoder
and CRC implementation. These are stop-and-wait tests; hardware timing and
independently generated full-duplex stress remain to be tested.

## Driver API and application use

The existing API is preserved: start() opens synchronously and may throw;
is_running() reports local I/O state; send() returns queue admission, not delivery.
wait_for_transmit_idle(timeout) now provides a bounded wait for outstanding local
writes. Call it only from the owning thread outside receive callbacks. It does
not acknowledge physical transmission or peer receipt. Do not stop immediately
after enqueueing the final reply; wait for local writes first.

Config::logger is optional and empty by default. To retain application telemetry,
initialize MCAP/Foxglove and set this callback in the application that includes
Telemetry.hpp:

```cpp
config.logger = [](auto message, const std::string& topic) {
    core::log(std::move(message), topic);
};
```

Keep logging and receive callbacks short. Stop UART before destroying their state.
An exception from logging does not prevent application delivery.

To send a production command, include DrivetrainUARTCommand.hpp and construct:

```cpp
auto command = comms::make_drivetrain_uart_command(
    {100.f, 200.f, 300.f, 400.f}, {1.f, 2.f, 3.f, 4.f});
hytech_msgs::MCUOutputData feedback_prototype;
comms::UARTComms uart(config, feedback_prototype, [](auto message) {
    // Process feedback outside the test wrapper contract.
});
uart.start();
if (!uart.send(command)) { /* stopped or TX capacity full */ }
```

## Teensy implementation contract

This branch's wire protocol is:

```text
COBS(protobuf payload | CRC-low | CRC-high) | 0x00
```

There is no required opening delimiter, kind byte, HELLO/ACK, schema fingerprint,
negotiated payload limit, reconnect loop or incomplete-frame timeout in this
branch. The handshake protocol described for abdur/uart is a different protocol;
do not implement it for this branch. Both endpoints must agree on message types,
schema revision and limits out of band. The test harness detects missing replies
using its own application deadline.

CRC-16/CCITT-FALSE covers only the serialized payload: polynomial 0x1021, init
0xffff, no reflection, xorout 0; check vector 123456789 -> 0x29b1. The two CRC
bytes are little-endian before COBS encoding. Ignore empty delimiter-separated
segments; reject malformed COBS, bad CRC and invalid Protobuf.
For payload limit P, decoded storage must accommodate P+2 bytes, and encoded
storage must accommodate (P+2)+floor((P+2)/254)+1 bytes excluding the delimiter.
Discard an overlong frame through the next delimiter, then resume parsing.

Implement these in vehicle-firmware:

1. A HardwareSerial adapter on a schematic-verified free UART. Serial1 defaults
   to Teensy RX pin 0/TX pin 1; board routing must be checked. Serial8 is already
   used by the maintenance-panel ADC. USB Serial remains diagnostics only.
2. Fixed-capacity RX/TX buffers and nonblocking framing. Consume available bytes,
   retain partial packets and partial writes, and bound work per scheduler tick.
   Use availableForWrite(); never wait indefinitely for a whole message.
3. nanopb generated from the same pinned base_msgs.proto, hytech_msgs.proto and
   uart_bench.proto. Put both source directories in the protoc import path, and
   this directory in the nanopb options search path; uart_bench.options bounds
   the pattern to 256 bytes. Use applicable HT_proto options too. Do not mix
   conflicting generated headers from the VCR's release tarball and newer sources.
4. Decode Command, require command/desired_rpms/torque_limit_nm presence, validate
   finite values and nonnegative torque limits, run ID and sequential sequence,
   and construct Reply with has_decoded and the nested nanopb presence flags set.
   Preserve pattern bytes. This echo must not reach actuators.
5. A vcr_uart_test environment with a dedicated main excluded from vcr-prod.
   After bench tests pass, schedule the UART service beside normal VCR tasks and
   deliberately map validated production commands into controller state.

The complete Linux class cannot be copied: Boost.Asio and full Protobuf are not
Teensy's peripheral interface. Share or port the wire algorithms, use HardwareSerial
for UART and nanopb for messages, and service them through the VCR scheduler.

## Existing VCR transports and production integration

Reference vehicle-firmware commit 5491883bcbaf9459adbc52037c1846d87172c531:

- CAN: FlexCAN_T4 handles the MCU controllers through board transceivers.
  VCRCANInterfaceImpl callbacks queue frames; main-loop code dispatches by ID.
  HT_CAN Unpack functions extract command values. DrivebrainInterface stores
  values and local receipt times; outgoing frames use Pack functions and TX queues.
  This is independent of Linux SocketCAN/CANComms.
- Ethernet: QNEthernet provides IP/UDP. VCREthernetInterface constructs telemetry;
  DrivebrainInterface calls handle_ethernet_socket_send_pb to nanopb-encode into
  a bounded buffer and send one UDP datagram. UDP has packet boundaries; UART
  needs the explicit framing above.

UART should follow the same peripheral -> buffer -> decode/validate -> interface
state -> controller separation. Choose an explicit source policy relative to CAN.
Do not silently overwrite both telemetry and auxiliary CAN inputs.

The current VCR permits 40 ms command age and at most 20 ms difference between
speed/torque-limit receipt times. Update both vectors' local receipt timestamps
atomically after validating a complete production command. Do not compare unsynced
Pi and VCR clocks. Malformed input must not refresh command age.
The current production schema has no run/sequence fields; robust replay rejection
needs an agreed application/schema extension before enabling UART control.
A detached GPIO cable can leave is_running() true; use freshness/deadlines.
The configurable TX FIFO can hold stale commands, so a production sender needs
bounded pending work, command deadlines and a chosen latest-command policy.

For initial wiring, cross Pi GPIO14/TX (physical 8) to VCR RX and Pi GPIO15/RX
(physical 10) to VCR TX, with a common logic ground. Use 3.3 V signals, approved
separate power supplies, short leads, a verified VCR connector pinout and inhibited
actuation. Repeat under real scheduler load and on the deployment image.
