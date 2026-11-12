# UART bench

The laptop coordinates the Pi over SSH and the Teensy over USB. The Pi and Teensy exchange UART test messages. Connect the Pi's GPIO 14 TX (header pin 8) to Teensy Serial1 RX (pin 0), GPIO 15 RX (header pin 10) to Serial1 TX (pin 1), and a ground pin to Teensy ground. Leave the Teensy connected to the laptop over USB.

From the Drivebrain root, prepare the coordinator once:

```sh
uv run tests/hil/uart/tester/uart_bench.py setup --no-flash
```

`--no-flash` verifies the already installed bench firmware; omit it only when you intend to flash the Teensy. The Pi must have GPIO UART enabled, its serial console disabled, and its user must have access to `/dev/ttyAMA0`.

Start a suite and choose an ID that you can reuse:

```sh
uv run tests/hil/uart/tester/uart_bench.py run --suite-id 1
```

If the connection drops, list the phases and resume at a name or 1-based index:

```sh
uv run tests/hil/uart/tester/uart_bench.py run --list-phases
uv run tests/hil/uart/tester/uart_bench.py run --suite-id 1 --start-at command-500hz
# Equivalent: --start-at 4
```

A fresh raw-message preflight runs before each invocation. The selected phase and every subsequent phase run again. Each phase replaces the same-named record as it starts; earlier phases and later phases not yet restarted retain their saved results. The suite passes only when every profile phase, preflight, and final recovery check has passed. A suite ID must reuse the same profile, baud, Teensy port, seed, and protocol schema; use a new ID for a different configuration.

Reports are under `.cache/uart-bench/reports/<suite-id>/`: `report.json` contains the current per-phase results and latest live snapshots, `report.md` is a summary, and `events.jsonl` appends timestamped events with attempt and phase identifiers. JSON reports are atomically updated throughout the run. Events preserve historical attempts even when a phase's current result is replaced. A disconnected or interrupted phase is marked aborted and can be restarted. The dashboard shows trends normalized to each recent window; RTT is the Pi-measured round trip, not one-way latency.

Use `--profile` for a different profile, `--port` for another Teensy UART, and `--pi-uart` if the Pi uses a different device. Use `--output` only when you want to choose the report directory yourself; reuse that same directory when resuming.
