"""Exercise the real UARTComms binary through two PTYs; no hardware or pyserial required."""
import binascii
import os
from pathlib import Path
import pty
import select
import subprocess
import sys
import tempfile
import time
import tty


def cobs_encode(data):
    encoded = bytearray([0])
    index, code = 0, 1
    for value in data:
        if value == 0:
            encoded[index] = code
            index, code = len(encoded), 1
            encoded.append(0)
        else:
            encoded.append(value)
            code += 1
            if code == 255:
                encoded[index] = code
                index, code = len(encoded), 1
                encoded.append(0)
    encoded[index] = code
    return b'\0' + encoded + b'\0'


def frame(body):
    return cobs_encode(body + binascii.crc_hqx(body, 0xffff).to_bytes(2, 'little'))


def decode(encoded):
    decoded = bytearray()
    index = 0
    while index < len(encoded):
        code = encoded[index]
        index += 1
        if not code or index + code - 1 > len(encoded):
            raise AssertionError('Driver emitted invalid COBS')
        decoded.extend(encoded[index:index + code - 1])
        index += code - 1
        if code != 255 and index < len(encoded):
            decoded.append(0)
    if len(decoded) < 3 or binascii.crc_hqx(decoded[:-2], 0xffff) != int.from_bytes(decoded[-2:], 'little'):
        raise AssertionError('Driver CRC disagrees with independent CRC-16/CCITT-FALSE implementation')
    return decoded[:-2]


def run(binary, name, pi_args=(), peer_args=(), *, faults=False, fragment=False,
        expected=(0, 0), count=20, drop_reply=False):
    masters, slaves = [], []
    processes, logs = [], []
    try:
        for _ in range(2):
            master, slave = pty.openpty()
            tty.setraw(slave)
            os.set_blocking(master, False)
            masters.append(master)
            slaves.append(slave)
        for i, (role, args) in enumerate((('pi', pi_args), ('peer', peer_args))):
            log = tempfile.TemporaryFile()
            logs.append(log)
            processes.append(subprocess.Popen(
                [binary, '--role', role, '--device', os.ttyname(slaves[i]),
                 '--count', str(count), '--interval-ms', '0', '--timeout-ms', '1800', *args],
                stdout=log, stderr=subprocess.STDOUT))
        pending = [bytearray(), bytearray()]
        injected = [False, False]
        limit = time.monotonic() + 20
        while any(p.poll() is None for p in processes):
            if time.monotonic() > limit:
                raise AssertionError(f'{name}: process hung')
            readable, _, _ = select.select(masters, [], [], .02)
            for source in readable:
                direction = masters.index(source)
                try:
                    chunk = os.read(source, 65536)
                except BlockingIOError:
                    continue
                pending[direction].extend(chunk)
                while 0 in pending[direction]:
                    end = pending[direction].index(0)
                    encoded = bytes(pending[direction][:end])
                    del pending[direction][:end + 1]
                    if not encoded:
                        continue
                    decode(encoded)  # Independently validate every emitted frame.
                    if drop_reply and direction == 1:
                        continue
                    packet = b'\0' + encoded + b'\0'
                    if faults and not injected[direction]:
                        corrupted = bytearray(packet)
                        corrupted[-2] ^= 0x40
                        # Invalid COBS, CRC corruption, CRC-valid invalid protobuf, then a valid frame.
                        packet = b'\0\x05\x01\0' + bytes(corrupted) + frame(b'\xff') + packet
                        injected[direction] = True
                    destination = masters[1 - direction]
                    parts = [packet[j:j+1] for j in range(len(packet))] if fragment else [packet]
                    for part in parts:
                        while part:
                            try:
                                written = os.write(destination, part)
                                part = part[written:]
                            except BlockingIOError:
                                select.select([], [destination], [], .05)
                        if fragment:
                            time.sleep(.0001)
        codes = tuple(p.wait() for p in processes)
        output = []
        for log in logs:
            log.seek(0)
            output.append(log.read().decode(errors='replace'))
        if codes != expected:
            raise AssertionError(f'{name}: exit codes {codes}, wanted {expected}\n' + '\n'.join(output))
        for code, text in zip(codes, output):
            if code == 0 and 'PASS role=' not in text:
                raise AssertionError(f'{name}: missing successful summary\n{text}')
        print(f'PASS {name}')
    finally:
        for process in processes:
            if process.poll() is None:
                process.kill()
            process.wait()
        for fd in masters + slaves:
            os.close(fd)
        for log in logs:
            log.close()


def main():
    binary = str(Path(sys.argv[1]).resolve())
    run(binary, '1000 command/reply exchanges', count=1000)
    run(binary, 'COBS boundary and binary data', ('--pattern-bytes', '254'), ('--max-payload', '400'))
    run(binary, 'default/zero-valued fields', ('--pattern-bytes', '0', '--rpms', '0,0,0,0', '--torque-limits', '0,0,0,0'))
    run(binary, 'logger exceptions preserve delivery', ('--throw-logger',), ('--throw-logger',), count=3)
    run(binary, 'corruption rejected and stream resynchronizes', faults=True)
    run(binary, 'fragmented byte stream', fragment=True, count=3)
    run(binary, 'peer receive limit enforced', peer_args=('--max-payload', '128'), expected=(1, 1))
    run(binary, 'local payload limit enforced', pi_args=('--max-payload', '128'), expected=(1, 1))
    run(binary, 'missing reply fails application deadline', drop_reply=True, expected=(1, 1), count=2)
    result = subprocess.run([binary, '--device', '/unused', '--torque-limits', '-1,0,0,0'], capture_output=True)
    assert result.returncode == 1 and b'nonnegative' in result.stderr
    print('PASS invalid torque CLI')


if __name__ == '__main__':
    main()
