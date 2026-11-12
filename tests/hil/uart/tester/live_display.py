"""Compact terminal dashboard and readable line-oriented output for logs."""
from collections import deque
import shutil
import sys


def sparkline(values):
    if not values:
        return '-'
    levels = '▁▂▃▄▅▆▇█'
    low, high = min(values), max(values)
    if high == low:
        return levels[0] * len(values)
    return ''.join(levels[min(7, int((v - low) * 7 / (high - low)))] for v in values)


class LiveDisplay:
    def __init__(self, stream=None):
        self.stream = stream or sys.stdout
        self.interactive = self.stream.isatty()
        self.lines = 0
        self.rates = deque(maxlen=18)
        self.latencies = deque(maxlen=18)
        self.phase = None

    def update(self, phase, seconds, value, firmware, rate):
        if phase != self.phase:
            self.rates.clear()
            self.latencies.clear()
            self.phase = phase
        elapsed = value.get('elapsed', 0)
        verified, missing = value.get('verified', 0), value.get('missing', 0)
        latency = value.get('rtt_p99_ms', 0)
        self.rates.append(rate)
        if verified:
            self.latencies.append(latency)
        width = shutil.get_terminal_size((100, 24)).columns
        if not self.interactive or width < 80:
            print(f"{phase} {elapsed:.1f}/{seconds:g}s TX {rate:.1f}/s accepted {value.get('accepted', 0)} "
                  f"verified {verified} missing {missing} mismatch {value.get('mismatch', 0)} "
                  f"RTT p99 {latency:.3f}ms Teensy CRC {firmware.get('crc_errors', 0)}",
                  file=self.stream, flush=True)
            self.lines = 0
            return
        # Keep all lines under 80 columns so wrapping cannot corrupt cursor positioning.
        inner = 76
        def row(text):
            return '| ' + text[:inner-2].ljust(inner-2) + ' |'
        border = '+' + '-' * inner + '+'
        fraction = min(1, max(0, elapsed / seconds))
        filled = round(fraction * 18)
        rates = sparkline(self.rates)
        rtts = sparkline(self.latencies)
        lines = [border,
                 row(f"UART BENCH  |  {phase}  |  {elapsed:.1f}/{seconds:g}s"),
                 border,
                 row(f"Progress [{'#' * filled}{'.' * (18-filled)}] {fraction*100:5.1f}%"),
                 row(f"TX rate   {rate:9.1f} msg/s      Trend {rates}"),
                 row(f"Accepted  {value.get('accepted',0):9d}             Sent {value.get('tx_frames',0):9d}"),
                 row(f"Verified  {verified:9d}             Missing {missing:9d}"),
                 row(f"Mismatch  {value.get('mismatch',0):9d}             Pending {value.get('pending',0):9d}"),
                 border,
                 row(f"RTT ms    p50 {value.get('rtt_p50_ms',0):7.3f}  p95 {value.get('rtt_p95_ms',0):7.3f}    Trend {rtts}"),
                 row(f"          p99 {latency:7.3f}  max {value.get('rtt_max_ms',0):7.3f}"),
                 border,
                 row(f"Errors    Pi CRC {value.get('rx_crc_errors',0):5d}  Teensy CRC {firmware.get('crc_errors',0):5d}  I/O {value.get('io_errors',0):5d}"),
                 row(f"          Queue rejected {value.get('queue_rejected',0):5d}  Decode {value.get('rx_cobs_errors',0)+value.get('rx_protobuf_errors',0):5d}"),
                 border]
        if self.lines:
            self.stream.write(f'\x1b[{self.lines}A')
        for line in lines:
            self.stream.write('\r\x1b[2K' + line + '\n')
        self.stream.flush()
        self.lines = len(lines)
