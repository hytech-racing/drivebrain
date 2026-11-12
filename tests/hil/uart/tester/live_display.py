"""A small terminal dashboard with plain output for logs and pipes."""
from collections import deque
import sys


def sparkline(values):
    if not values:
        return "—"
    levels = "▁▂▃▄▅▆▇█"
    low, high = min(values), max(values)
    if high == low:
        return levels[0] * len(values)
    return "".join(levels[min(7, int((value - low) * 7 / (high - low)))] for value in values)


class LiveDisplay:
    def __init__(self, stream=None):
        self.stream = stream or sys.stdout
        self.interactive = self.stream.isatty()
        self.lines = 0
        self.rates = deque(maxlen=32)
        self.latencies = deque(maxlen=32)

    def update(self, phase, seconds, value, firmware, rate):
        elapsed = value.get("elapsed", 0)
        verified, missing = value.get("verified", 0), value.get("missing", 0)
        latency = value.get("rtt_p99_ms", 0)
        self.rates.append(rate)
        if verified:
            self.latencies.append(latency)
        if not self.interactive:
            print(f"{phase} {elapsed:.1f}/{seconds:g}s TX {rate:.1f}/s "
                  f"accepted {value.get('accepted', 0)} verified {verified} "
                  f"missing {missing} mismatch {value.get('mismatch', 0)} "
                  f"RTT p99 {latency:.3f}ms Teensy CRC {firmware.get('crc_errors', 0)}",
                  file=self.stream, flush=True)
            return
        fraction = min(1, max(0, elapsed / seconds))
        width = 24
        filled = round(fraction * width)
        lines = [
            f"UART BENCH  ·  {phase}",
            f"[{'█' * filled}{'░' * (width - filled)}] {elapsed:6.1f} / {seconds:g} s",
            f"TX {rate:8.1f}/s  {sparkline(self.rates)}   accepted {value.get('accepted', 0)}  sent {value.get('tx_frames', 0)}",
            f"Verified {verified}   missing {missing}   mismatch {value.get('mismatch', 0)}   pending {value.get('pending', 0)}",
            f"RTT ms  p50 {value.get('rtt_p50_ms', 0):.3f}  p95 {value.get('rtt_p95_ms', 0):.3f}  p99 {latency:.3f}  max {value.get('rtt_max_ms', 0):.3f}  {sparkline(self.latencies)}",
            f"Errors  Pi CRC {value.get('rx_crc_errors', 0)}  Teensy CRC {firmware.get('crc_errors', 0)}  queue rejected {value.get('queue_rejected', 0)}  I/O {value.get('io_errors', 0)}",
        ]
        if self.lines:
            self.stream.write(f"\x1b[{self.lines}A")
        for line in lines:
            self.stream.write("\r\x1b[2K" + line + "\n")
        self.stream.flush()
        self.lines = len(lines)
