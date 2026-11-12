#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.10"
# dependencies = ["paramiko>=3.4,<5", "pyserial>=3.5,<4", "nanopb==0.4.9.1", "protobuf>=5.29,<8"]
# ///
"""Laptop coordinator: setup once, then dispatch/monitor repeatable UART runs."""
import argparse
from datetime import datetime, timezone
import getpass
import hashlib
import json
import os
import re
from pathlib import Path
import secrets
import shlex
import subprocess
import sys
import tarfile
import tempfile
import time
from pi_connection import connection_failure, guard_pi_connection
from live_display import LiveDisplay
from suite_report import Report

BENCH = Path(__file__).resolve().parents[1]
DB = BENCH.parents[2]
DEFAULT_FW = DB.parent / "vehicle-firmware"
CACHE = DB / ".cache/uart-bench"
STATE_FILE = CACHE / "setup.json"


def shell(*args):
    return shlex.join(str(x) for x in args)


@guard_pi_connection
def read_pi_channel(channel, stderr=False):
    return channel.recv_stderr(65536) if stderr else channel.recv(65536)


def local(command, cwd=None):
    print("+", shlex.join(map(str, command)), flush=True)
    subprocess.run(list(map(str, command)), cwd=cwd, check=True)


def git_info(path):
    def git(*args):
        return subprocess.check_output(["git", "-C", str(path), *args], text=True).strip()
    return {"commit": git("rev-parse", "HEAD"), "dirty": bool(git("status", "--porcelain"))}


class SSH:
    def __init__(self, target, password_env="HYTECH_PI_PASSWORD"):
        import paramiko
        self.paramiko = paramiko
        self.target = target
        self.user, self.host = target.split("@", 1)
        self.password = os.environ.get(password_env)
        self.client = None

    @guard_pi_connection
    def connect(self):
        if self.client:
            self.client.close()
        self.client = self.paramiko.SSHClient()
        self.client.load_system_host_keys()
        # Trust follows ~/.ssh/known_hosts. Establish a first connection with ssh
        # yourself; this tool never disables host-key verification.
        self.client.set_missing_host_key_policy(self.paramiko.RejectPolicy())
        try:
            self.client.connect(self.host, username=self.user, password=self.password,
                                timeout=5, auth_timeout=10, banner_timeout=5,
                                look_for_keys=self.password is None, allow_agent=self.password is None)
        except (self.paramiko.AuthenticationException, self.paramiko.SSHException) as exc:
            if not isinstance(exc, self.paramiko.AuthenticationException) and "No authentication methods available" not in str(exc):
                if "Error reading SSH protocol banner" in str(exc):
                    raise connection_failure("Pi SSH connection did not respond") from exc
                raise
            if self.password is not None:
                raise
            self.client.close()
            self.password = getpass.getpass(f"SSH password for {self.target}: ")
            self.client.connect(self.host, username=self.user, password=self.password,
                                timeout=5, auth_timeout=10, banner_timeout=5,
                                look_for_keys=False, allow_agent=False)
        self.client.get_transport().set_keepalive(60)

    @guard_pi_connection
    def start(self, command):
        transport = self.client.get_transport()
        if not transport or not transport.is_active():
            raise ConnectionError("SSH transport is not connected")
        try:
            channel = transport.open_session(timeout=5)
            channel.exec_command(command)
        except self.paramiko.SSHException as exc:
            if not transport.is_active():
                raise connection_failure("SSH transport disconnected") from exc
            raise
        return channel

    @guard_pi_connection
    def execute(self, command, timeout=300, sudo=False, stream=False):
        if sudo:
            command = "sudo -S -p '' sh -c " + shlex.quote(command)
        channel = self.start(command)
        if sudo:
            if self.password is None:
                self.password = getpass.getpass(f"sudo password for {self.target}: ")
            channel.sendall((self.password + "\n").encode())
        channel.shutdown_write()
        output, errors = bytearray(), bytearray()
        deadline = time.monotonic() + timeout
        while True:
            if not self.client.get_transport().is_active():
                raise connection_failure("SSH transport disconnected")
            for ready, read, data in [(channel.recv_ready, channel.recv, output),
                                      (channel.recv_stderr_ready, channel.recv_stderr, errors)]:
                if ready():
                    chunk = read(65536)
                    data.extend(chunk)
                    if stream:
                        print(chunk.decode(errors="replace"), end="", flush=True)
            if channel.exit_status_ready() and not channel.recv_ready() and not channel.recv_stderr_ready():
                break
            if time.monotonic() >= deadline:
                channel.close()
                raise connection_failure("Pi command did not respond before its deadline")
            time.sleep(.02)
        code = channel.recv_exit_status()
        if code:
            raise RuntimeError(f"remote command exited {code}: " + errors.decode(errors="replace")[-3000:] + output.decode(errors="replace")[-1000:])
        return output.decode(errors="replace")

    def close(self):
        if self.client:
            self.client.close()


def teensy_device(explicit=None):
    if explicit:
        return explicit
    from serial.tools import list_ports
    devices = [p.device for p in list_ports.comports() if p.vid == 0x16C0]
    if len(devices) != 1:
        raise RuntimeError(f"Expected one Teensy USB serial device; found {devices}. Supply --teensy-device.")
    return devices[0]


class Teensy:
    def __init__(self, device=None):
        import serial
        self.device = teensy_device(device)
        self.serial = serial.Serial(self.device, 115200, timeout=0, write_timeout=2)
        self.buffer = bytearray()

    def poll(self):
        self.buffer.extend(self.serial.read(4096))
        if len(self.buffer) > 16384:
            self.buffer.clear()
            raise RuntimeError("Teensy USB control output exceeds limit")
        messages = []
        while b"\n" in self.buffer:
            line, _, rest = self.buffer.partition(b"\n")
            self.buffer[:] = rest
            try:
                value = json.loads(line)
            except (ValueError, UnicodeDecodeError):
                continue  # Opening USB may start in the middle of a status line.
            if isinstance(value, dict):
                messages.append(value)
        return messages

    def command(self, text, event, timeout=5):
        self.poll()
        self.serial.write((text + "\n").encode())
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for message in self.poll():
                if message.get("event") == "control_error":
                    raise RuntimeError("Teensy rejected USB configuration")
                if message.get("event") == event:
                    return message
            time.sleep(.01)
        raise TimeoutError(f"Teensy did not acknowledge {text.split()[0]}")

    def close(self):
        self.serial.close()


def identity_check(identity, digest):
    if identity.get("version") != 1 or identity.get("schema_sha256") != digest:
        raise RuntimeError("Pi/Teensy protocol or schema identity mismatch; run setup again")


def source_archive(ht_proto):
    files = [DB / "drivebrain_comms/include/UARTComms.hpp", DB / "drivebrain_comms/src/UARTComms.cpp"]
    files += [p for p in BENCH.rglob("*") if p.is_file() and "__pycache__" not in p.parts]
    files += [ht_proto / name for name in ("base_msgs.proto", "hytech_msgs.proto", "hytech_msgs.options") if (ht_proto / name).exists()]
    digest = hashlib.sha256()
    entries = []
    for path in sorted(files):
        if path.is_relative_to(DB):
            relative = Path("drivebrain") / path.relative_to(DB)
        else:
            relative = Path("HT_proto/proto") / path.name
        data = path.read_bytes()
        digest.update(str(relative).encode() + b"\0" + data)
        entries.append((path, relative))
    return entries, digest.hexdigest()


def setup(args):
    ht_proto = args.ht_proto.resolve()
    firmware = args.firmware.resolve()
    generated = firmware / ".pio/uart_bench_generated"
    local([sys.executable, BENCH / "generate_messages.py", "--ht-proto", ht_proto, "--out", generated, "--nanopb"])
    manifest = json.loads((generated / "manifest.json").read_text())
    entries, digest = source_archive(ht_proto)
    ssh = SSH(args.pi, args.password_env)
    try:
        ssh.connect()
        if args.install_deps:
            ssh.execute("apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y "
                        "build-essential cmake libboost-dev libprotobuf-dev protobuf-compiler libspdlog-dev python3",
                        sudo=True, stream=True, timeout=900)
        home = ssh.execute("printf '%s' \"$HOME\"").strip()
        root = f"{home}/.cache/hytech-uart-bench/{digest[:16]}"
        ssh.execute(shell("mkdir", "-p", root + "/src"))
        with tempfile.TemporaryDirectory() as temp:
            archive = Path(temp) / "sources.tar.gz"
            with tarfile.open(archive, "w:gz") as tar:
                for path, relative in entries:
                    tar.add(path, arcname=str(relative), recursive=False)
            with ssh.client.open_sftp() as sftp:
                sftp.put(str(archive), root + "/sources.tar.gz")
        ssh.execute(shell("tar", "-xzf", root + "/sources.tar.gz", "-C", root + "/src"))
        ssh.execute(shell("cmake", "-S", root + "/src/drivebrain/tests/hil/uart", "-B", root + "/build",
                          "-DHT_PROTO_DIR=" + root + "/src/HT_proto/proto"), stream=True)
        ssh.execute(shell("cmake", "--build", root + "/build", "-j2"), stream=True, timeout=900)
        executable = root + "/build/uart_stress_peer"
        identity_check(json.loads(ssh.execute(shell(executable, "--identity"))), manifest["schema_sha256"])
        pi_metadata = ssh.execute("uname -a\ncat /etc/os-release\nvcgencmd get_throttled 2>/dev/null || true")
        local(["uv", "tool", "run", "--from", "platformio==6.2.0", "pio", "run", "-e", "vcr_uart_test"], cwd=firmware)
        if not args.no_flash:
            loader = Path.home() / ".platformio/packages/tool-teensy/teensy_loader_cli"
            subprocess.run([str(loader), "--mcu=TEENSY41", "-w", "-s", "-v",
                            str(firmware / ".pio/build/vcr_uart_test/firmware.hex")], check=True, timeout=45)
        deadline = time.monotonic() + 10
        while True:
            try:
                device = teensy_device(args.teensy_device)
                break
            except RuntimeError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.25)
        teensy = Teensy(device)
        try:
            identity_check(teensy.command("HELLO", "identity"), manifest["schema_sha256"])
        finally:
            teensy.close()
        state = {"pi": args.pi, "executable": executable, "schema_sha256": manifest["schema_sha256"],
                 "source_sha256": digest, "teensy_device": device, "pi_uart": args.pi_uart,
                 "drivebrain": git_info(DB), "firmware": git_info(firmware), "pi_metadata": pi_metadata,
                 "created_at": datetime.now(timezone.utc).isoformat()}
        CACHE.mkdir(parents=True, exist_ok=True)
        STATE_FILE.write_text(json.dumps(state, indent=2) + "\n")
        print(f"Setup verified: Pi executable built; Teensy firmware identity matched. State: {STATE_FILE}")
    finally:
        ssh.close()


def phase_errors(phase, result, firmware):
    errors = []
    if result.get("event") != "result":
        return ["Pi phase did not produce a final result"]
    if result.get("errors", 1) or result.get("interrupted"):
        errors.append("Pi reported unexpected errors or interruption")
    if not result.get("verified"):
        errors.append("No UART replies were verified")
    kind = phase["kind"]
    normal = kind not in ("faults", "overload")
    if normal:
        for key in ("cobs_errors", "crc_errors", "oversize_errors", "protobuf_errors", "invalid_requests", "invalid_commands", "tx_busy"):
            if firmware.get(key, 0):
                errors.append(f"Teensy {key}={firmware[key]}")
        expected = result.get("verified", 0)
        if firmware.get("commands", 0) + firmware.get("patterns", 0) != expected:
            errors.append("Teensy decoded count differs from verified replies")
    elif kind == "faults":
        # Short custom runs need enough time to cycle through all six injections.
        if result.get("injected", 0) >= 6:
            for key in ("cobs_errors", "crc_errors", "oversize_errors", "protobuf_errors", "invalid_commands"):
                if firmware.get(key, 0) == 0:
                    errors.append(f"Injected {key} was not observed")
        if result.get("injected", 0) == 0:
            errors.append("No deliberate faults were injected")
    else:
        if not (result.get("queue_rejected", 0) or result.get("missing", 0) or firmware.get("tx_busy", 0)):
            errors.append("Requested overload did not exercise backpressure")
    return errors


def wire_mode(kind):
    return "raw" if kind in ("raw", "command", "overload") else "bench"


def dispatch(ssh, teensy, state, phase, args, report, baud, seed):
    phase = dict(phase)
    if phase["kind"] == "pattern":
        sizes = json.loads(ssh.execute(shell(state["executable"], "--describe", "--kind", "pattern",
                                             "--pattern-bytes", phase.get("pattern_bytes", 0))))
        cap = max(1, int(baud / 10 * .70 / max(sizes.values())))
        phase["requested_rate"] = phase["rate"]
        phase["rate"] = min(phase["rate"], cap)
        phase["wire_size_bounds"] = sizes
    run_id = secrets.randbits(64) or 1
    mode = wire_mode(phase["kind"])
    configured = teensy.command(f"CONFIG {args.port} {baud} {mode} {run_id}", "configured")
    identity_check(configured, state["schema_sha256"])
    if int(configured["run_id"]) != run_id or configured["port"] != args.port or configured["baud"] != baud:
        raise RuntimeError("Teensy configuration readback differs")
    command = [state["executable"], "--device", args.pi_uart or state["pi_uart"], "--phase", phase["name"],
               "--kind", phase["kind"], "--seconds", str(phase["seconds"]), "--rate", str(phase["rate"]),
               "--baud", str(baud), "--run-id", str(run_id), "--seed", str(seed),
               "--pattern-bytes", str(phase.get("pattern_bytes", 0)), "--cpu-workers", str(phase.get("cpu_workers", 0))]
    channel = ssh.start("exec " + shell(*command))
    channel.shutdown_write()
    buffer = bytearray()
    final = None
    deadline = time.monotonic() + phase["seconds"] + 15
    last_firmware = configured
    previous_count, previous_elapsed = 0, 0
    display = report.display
    try:
        while True:
            for value in teensy.poll():
                identity_check(value, state["schema_sha256"])
                if int(value.get("run_id", 0)) != run_id:
                    raise RuntimeError("Teensy reset or session changed during phase")
                last_firmware = value
                report.event("teensy", value)
            if channel.recv_ready():
                buffer.extend(read_pi_channel(channel))
            while b"\n" in buffer:
                line, _, rest = buffer.partition(b"\n")
                buffer[:] = rest
                try:
                    value = json.loads(line)
                except (ValueError, UnicodeDecodeError):
                    report.event("pi-log", line.decode(errors="replace"))
                    continue
                report.event("pi", value)
                if value.get("event") == "fatal":
                    raise RuntimeError(value.get("message", "Pi fatal error"))
                identity_check({"version": 1, **value}, state["schema_sha256"])
                if value.get("event") == "result":
                    final = value
                elapsed = value.get("elapsed", 0)
                count = value.get("accepted", 0)
                actual = (count - previous_count) / max(.001, elapsed - previous_elapsed)
                previous_count, previous_elapsed = count, elapsed
                display.update(phase['name'], phase['seconds'], value, last_firmware, actual)
            if channel.recv_stderr_ready():
                report.event("pi-stderr", read_pi_channel(channel, stderr=True).decode(errors="replace"))
            if channel.exit_status_ready() and not channel.recv_ready():
                break
            if not ssh.client.get_transport().is_active():
                raise connection_failure("SSH transport disconnected")
            if time.monotonic() >= deadline:
                raise connection_failure("Pi phase did not respond before its deadline")
            time.sleep(.01)
        code = channel.recv_exit_status()
        if final is None:
            if not ssh.client.get_transport().is_active() or code == -1:
                raise connection_failure("SSH connection closed before the Pi returned results")
            raise RuntimeError(f"Pi exited {code} without a final report")
        firmware = teensy.command("STATUS", "snapshot")
        identity_check(firmware, state["schema_sha256"])
        errors = phase_errors(phase, final, firmware)
        if code != 0 and not errors:
            errors.append(f"Pi process exited {code}")
        return {"name": phase["name"], "config": phase, "run_id": str(run_id), "pi": final,
                "teensy": firmware, "errors": errors, "passed": not errors}
    finally:
        channel.close()


def validate_profile(profile):
    if not isinstance(profile, dict) or not isinstance(profile.get("phases"), list) or not profile["phases"]:
        raise ValueError("profile must contain nonempty phases")
    if not 9600 <= int(profile.get("baud", 921600)) <= 3000000:
        raise ValueError("invalid profile baud")
    names = set()
    for phase in profile["phases"]:
        if phase["name"] in names or phase["kind"] not in ("command", "pattern", "faults", "overload"):
            raise ValueError("duplicate phase name or unsupported kind")
        names.add(phase["name"])
        if not 0 < float(phase["seconds"]) <= 86400 or not 0 < float(phase["rate"]) <= 100000:
            raise ValueError("invalid duration/rate")
        if not 0 <= int(phase.get("pattern_bytes", 0)) <= 1024 or not 0 <= int(phase.get("cpu_workers", 0)) <= 4:
            raise ValueError("invalid pattern/CPU configuration")
    return profile


def run(args):
    state = json.loads(STATE_FILE.read_text())
    profile = validate_profile(json.loads(args.profile.read_text()))
    baud = args.baud or profile.get("baud", 921600)
    if not 9600 <= baud <= 3000000:
        raise ValueError("baud must be between 9600 and 3000000")
    seed = profile.get("seed", 1979)
    suite_id = args.suite_id or (datetime.now().strftime("%Y%m%d-%H%M%S") + "-" + secrets.token_hex(3))
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}", suite_id) or suite_id in (".", ".."):
        raise ValueError("suite ID must contain only letters, numbers, dots, underscores or hyphens")
    names = [phase["name"] for phase in profile["phases"]]
    start = args.start_at or names[0]
    if start.isdecimal():
        index = int(start) - 1
        if not 0 <= index < len(names):
            raise ValueError(f"start index must be between 1 and {len(names)}")
    elif start in names:
        index = names.index(start)
    else:
        raise ValueError(f"unknown start phase {start!r}; use --list-phases")
    output = args.output or CACHE / "reports" / suite_id
    print(f"Suite ID: {suite_id}  |  Starting: {names[index]}  |  Reports: {output}", flush=True)
    report = Report(output, {"setup": state, "profile": profile, "port": args.port, "baud": baud, "seed": seed})
    report.display = LiveDisplay()
    ssh = SSH(args.pi or state["pi"], args.password_env)
    teensy = None
    try:
        ssh.connect()
        identity_check(json.loads(ssh.execute(shell(state["executable"], "--identity"))), state["schema_sha256"])
        teensy = Teensy(args.teensy_device)
        identity_check(teensy.command("HELLO", "identity"), state["schema_sha256"])
        preflight = {"name": "production-payload-check", "kind": "raw", "seconds": 1.5, "rate": 20}
        report.phase_begin(preflight)
        result = dispatch(ssh, teensy, state, preflight, args, report, baud, seed)
        report.put(result)
        if not result["passed"]:
            raise RuntimeError("Raw-payload preflight failed; check UART configuration and TX/RX/GND wiring")
        for phase in profile["phases"][index:]:
            report.phase_begin(phase)
            try:
                result = dispatch(ssh, teensy, state, phase, args, report, baud, seed)
            except Exception as exc:
                result = next(p for p in report.data["phases"] if p["name"] == phase["name"])
                result.update(passed=False, errors=[str(exc)], aborted=True, status="aborted")
                report.put(result)
                report.display.lines = 0
                print(f"ERROR: {phase['name']}: {exc}", file=sys.stderr, flush=True)
                recovered = False
                recovery_error = None
                for attempt in range(3):
                    report.event("recovery", {"attempt": attempt + 1, "error": str(exc)})
                    try:
                        ssh.connect()
                        teensy.close()
                        teensy = Teensy(args.teensy_device)
                        identity_check(teensy.command("HELLO", "identity"), state["schema_sha256"])
                        recovered = True
                        break
                    except Exception as recovery_exc:
                        recovery_error = recovery_exc
                        time.sleep(1)
                if not recovered:
                    raise RuntimeError(f"Endpoints did not recover after three attempts: {recovery_error}")
            report.put(result)
        # A final healthy exchange proves recovery after intentional overload.
        recovery_phase = {**preflight, "name": "post-overload-recovery"}
        report.phase_begin(recovery_phase)
        result = dispatch(ssh, teensy, state, recovery_phase, args, report, baud, seed)
        report.put(result)
        report.data["complete"] = True
    except KeyboardInterrupt:
        report.data["errors"].append("Interrupted by user; partial results saved")
    except Exception as exc:
        report.data["errors"].append(str(exc))
        print("ERROR:", exc, file=sys.stderr)
    finally:
        if teensy:
            try:
                teensy.command("STOP", "stopped", timeout=1)
            except Exception:
                pass
            teensy.close()
        ssh.close()
        report.data["finished_at"] = datetime.now(timezone.utc).isoformat()
        report.data["attempts"][-1].update(finished_at=report.data["finished_at"], errors=list(report.data["errors"]))
        for phase in report.data["phases"]:
            if phase.get("status") == "running" and phase.get("attempt") == report.attempt:
                phase.update(status="aborted", aborted=True, passed=False)
        report.save()
        report.close()
        print("Reports:", output)
    return 0 if report.data["passed"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="action", required=True)
    setup_parser = sub.add_parser("setup", help="Build Pi binary and build/flash Teensy endpoint")
    setup_parser.add_argument("--pi", default="user@user.local")
    setup_parser.add_argument("--ht-proto", type=Path, default=DB.parent / "HT_proto/proto")
    setup_parser.add_argument("--firmware", type=Path, default=DEFAULT_FW)
    setup_parser.add_argument("--pi-uart", default="/dev/ttyAMA0")
    setup_parser.add_argument("--install-deps", action="store_true")
    setup_parser.add_argument("--no-flash", action="store_true")
    run_parser = sub.add_parser("run", help="Dispatch the suite, show live stats, save reports")
    run_parser.add_argument("--pi")
    run_parser.add_argument("--pi-uart")
    run_parser.add_argument("--port", type=int, choices=range(1, 9), default=1)
    run_parser.add_argument("--baud", type=int)
    run_parser.add_argument("--profile", type=Path, default=BENCH / "profiles/default.json")
    run_parser.add_argument("--output", type=Path)
    run_parser.add_argument("--suite-id", help="Stable ID for resuming or replacing phase results")
    run_parser.add_argument("--start-at", help="Phase name or 1-based index to start from")
    run_parser.add_argument("--list-phases", action="store_true", help="List phase names and indices")
    for target in (setup_parser, run_parser):
        target.add_argument("--teensy-device")
        target.add_argument("--password-env", default="HYTECH_PI_PASSWORD", help="Optional environment variable; never stored")
    args = parser.parse_args()
    if args.action == "setup":
        setup(args)
        return 0
    if args.list_phases:
        profile = validate_profile(json.loads(args.profile.read_text()))
        for index, phase in enumerate(profile["phases"], 1):
            print(f"{index:2d}  {phase['name']}")
        return 0
    return run(args)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print("ERROR:", exc, file=sys.stderr)
        raise SystemExit(2)
