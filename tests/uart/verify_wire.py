"""Check real UARTComms output against independent COBS/CRC implementations."""
import binascii
from pathlib import Path
import subprocess
import sys

subprocess.run([sys.argv[1], "--fixtures", sys.argv[2]], check=True)


def decode(frame):
    assert frame[-1] == 0 and 0 not in frame[:-1]
    encoded = frame[:-1]
    out = bytearray()
    pos = 0
    while pos < len(encoded):
        code = encoded[pos]
        pos += 1
        assert code and pos + code - 1 <= len(encoded)
        out.extend(encoded[pos:pos + code - 1])
        pos += code - 1
        if code != 255 and pos < len(encoded):
            out.append(0)
    assert int.from_bytes(out[-2:], "little") == binascii.crc_hqx(out[:-2], 0xFFFF)
    return bytes(out[:-2])


fixtures = Path(sys.argv[2])
for size in (0, 1, 253, 254, 255, 256, 512, 1024):
    assert decode((fixtures / f"pattern-{size}.frame").read_bytes()) == bytes(i % 256 for i in range(size))
# Independently specified protobuf: nested vectors containing fixed32 floats.
import struct
def vector(values):
    return b"".join(bytes([(i + 1) * 8 + 5]) + struct.pack("<f", x) for i, x in enumerate(values))
rpm = vector((100, 200, -300, 400))
torque = vector((1, 2, 3, 4))
expected = b"\x0a\x14" + rpm + b"\x12\x14" + torque + b"\x18\xc0\xc4\x07"
assert decode((fixtures / "command.frame").read_bytes()) == expected
assert decode((fixtures / "zero.frame").read_bytes()) == b"\x0a\x00\x12\x00"
print("Independent wire checks passed")
