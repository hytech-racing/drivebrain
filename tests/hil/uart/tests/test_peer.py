"""Exercise the actual C++ peer through an independent virtual UART endpoint."""
import binascii
import json
import os
from pathlib import Path
import select
import subprocess
import sys
import threading
import tty
import unittest


def encode(payload):
    data = payload + binascii.crc_hqx(payload, 0xFFFF).to_bytes(2, "little")
    result = bytearray([0])
    code_at, code = 0, 1
    for value in data:
        if value == 0:
            result[code_at] = code
            code_at, code = len(result), 1
            result.append(0)
        else:
            result.append(value)
            code += 1
            if code == 255:
                result[code_at] = code
                code_at, code = len(result), 1
                result.append(0)
    result[code_at] = code
    return bytes(result) + b"\0"


def decode(encoded):
    result = bytearray()
    pos = 0
    while pos < len(encoded):
        code = encoded[pos]
        pos += 1
        if not code or pos + code - 1 > len(encoded):
            raise ValueError("invalid COBS")
        result.extend(encoded[pos:pos + code - 1])
        pos += code - 1
        if code != 255 and pos < len(encoded):
            result.append(0)
    if len(result) < 2 or int.from_bytes(result[-2:], "little") != binascii.crc_hqx(result[:-2], 0xFFFF):
        raise ValueError("invalid CRC")
    return bytes(result[:-2])


class PeerTest(unittest.TestCase):
    binary = None

    def exchange(self, kind, damage=None, size=0, cpu=0):
        master, slave = os.openpty()
        tty.setraw(slave)
        stopped = threading.Event()
        received = []
        errors = []

        def endpoint():
            pending = bytearray()
            try:
                while not stopped.is_set():
                    if not select.select([master], [], [], .05)[0]:
                        continue
                    for value in os.read(master, 4096):
                        if value:
                            pending.append(value)
                            continue
                        if not pending:
                            continue
                        payload = decode(bytes(pending))
                        pending.clear()
                        received.append(payload)
                        # MCUCommandData starts with field 1 as a length-delimited
                        # desired_rpms message; a bench Request starts with fixed64.
                        if kind in ("command", "raw", "overload") and payload[:1] != b"\x0a":
                            errors.append("expected a direct MCUCommandData payload")
                        if damage == "drop" and len(received) == 1:
                            continue
                        # The test schemas intentionally use matching tags 1..5.
                        # Independent decode/re-encoding checks the real serial driver.
                        response = encode(payload)
                        if damage == "duplicate" and len(received) == 1:
                            response += response
                        for start in range(0, len(response), 7):
                            os.write(master, response[start:start + 7])
            except (OSError, ValueError) as exc:
                if not stopped.is_set():
                    errors.append(str(exc))

        thread = threading.Thread(target=endpoint)
        thread.start()
        try:
            result = subprocess.run([str(self.binary), "--device", os.ttyname(slave), "--kind", kind,
                                     "--seconds", ".3", "--rate", "200", "--baud", "115200", "--pattern-bytes", str(size),
                                     "--cpu-workers", str(cpu)], capture_output=True, text=True, timeout=6)
            events = [json.loads(line) for line in result.stdout.splitlines() if line.startswith("{")]
            final = events[-1]
            self.assertEqual(final["event"], "result", result.stdout + result.stderr)
            self.assertGreater(final["verified"], 0)
            self.assertFalse(errors, errors)
            if damage:
                self.assertNotEqual(result.returncode, 0)
                self.assertGreater(final["errors"], 0)
                self.assertGreater(final["missing" if damage == "drop" else "duplicate"], 0)
            else:
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                self.assertEqual(final["errors"], 0)
                self.assertEqual(final["accepted"], final["verified"])
        finally:
            stopped.set()
            thread.join(2)
            os.close(master)
            os.close(slave)

    def test_commands(self): self.exchange("command")
    def test_production_schema(self): self.exchange("raw")
    def test_binary_pattern_boundaries(self):
        for size in (0, 254, 1024):
            with self.subTest(size=size): self.exchange("pattern", size=size)
    def test_cpu_load(self): self.exchange("command", cpu=2)
    def test_missing_reply_fails(self): self.exchange("command", damage="drop")
    def test_duplicate_reply_fails(self): self.exchange("command", damage="duplicate")


if __name__ == "__main__":
    PeerTest.binary = Path(sys.argv.pop(1)).resolve()
    unittest.main()
