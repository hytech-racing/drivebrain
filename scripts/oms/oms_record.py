#!/usr/bin/env python3
"""Record the Sensoric OMS Race raw TCP stream (port 51000) alongside the
labelled values the web UI shows (its MEAS command). Runs on the Jetson,
stdlib only. Both logs carry the sensor's own SampleTime, which is the join key.

    nohup python3 oms_record.py > /dev/null 2>&1 &     # run until pkill -INT -f oms_record.py
    python3 oms_record.py --duration 600               # or stop after 10 min

Output: <out>/<YYYYmmdd_HHMMSS>/{raw_51000.bin, packets.csv, meas.csv, log.txt}
"""
import argparse
import csv
import math
import os
import signal
import socket
import struct
import threading
import time
import urllib.request
from datetime import datetime

MAGIC = b"eS\x01\x00"
HDR = struct.Struct("<HHIIHH")  # magic, version, seq, len, type, frame_size
MEAS_COLS = ["t_epoch", "sampletime", "vel_kmh", "angle_deg", "dist_m", "pitch_deg", "roll_deg",
             "temp_sensor", "temp_ecu_pro", "temp_ecu", "ba_state", "runtime_current", "runtime_global", "raw_hex"]

stop = threading.Event()


def log(fh, msg):
    line = "%s %s" % (datetime.now().strftime("%H:%M:%S.%f")[:-3], msg)
    print(line, flush=True)
    fh.write(line + "\n")
    fh.flush()


def web_cmd(host, cmd):
    """Send one 4-char read command the way the web UI does and return the raw reply.
    ASCII fields are byte-reversed on the wire ("So" -> b"oS", "MEAS" -> b"SAEM")."""
    body = b"oS" + struct.pack("<HI", 1, 4) + cmd[::-1].encode()
    req = urllib.request.Request("http://%s/ajax" % host, data=body, method="POST",
                                 headers={"Content-Type": "application/octet-stream", "cache-control": "no-cache"})
    with urllib.request.urlopen(req, timeout=2) as r:
        return r.read()


def tcp_thread(host, port, outdir, logf, stats):
    raw = open(os.path.join(outdir, "raw_51000.bin"), "ab", buffering=0)
    idxf = open(os.path.join(outdir, "packets.csv"), "w", newline="")
    idx = csv.writer(idxf)
    idx.writerow(["t_epoch", "byte_offset", "seq", "n_frames", "first_sampletime", "last_sampletime"])
    written = 0
    while not stop.is_set():
        try:
            s = socket.create_connection((host, port), timeout=3)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 << 20)
            s.settimeout(2)
            log(logf, "tcp connected %s:%d" % (host, port))
        except OSError as e:
            log(logf, "tcp connect failed: %s" % e)
            stop.wait(1)
            continue
        buf = b""
        last_st = None
        try:
            while not stop.is_set():
                data = s.recv(1 << 16)
                if not data:
                    raise ConnectionError("closed by sensor")
                t = time.time()
                raw.write(data)
                written += len(data)
                buf += data
                # index every complete packet; buf always ends at the newest byte
                while True:
                    i = buf.find(MAGIC)
                    if i < 0:
                        buf = buf[-3:]
                        break
                    buf = buf[i:]
                    if len(buf) < HDR.size:
                        break
                    _, _, seq, ln, _, fs = HDR.unpack_from(buf)
                    total = 12 + ln
                    if len(buf) < total:
                        break
                    nfr = (ln - 4) // fs
                    st0 = struct.unpack_from("<f", buf, 16)[0]
                    st1 = struct.unpack_from("<f", buf, 16 + (nfr - 1) * fs)[0]
                    idx.writerow(["%.6f" % t, written - len(buf), seq, nfr, "%.3f" % st0, "%.3f" % st1])
                    if last_st is not None:
                        stats["dropped"] += max(0, round((st0 - last_st) * 1000) - 1)
                    last_st = st1
                    stats["packets"] += 1
                    stats["frames"] += nfr
                    stats["last_st"] = st1
                    buf = buf[total:]
                idxf.flush()
        except (OSError, ConnectionError) as e:
            log(logf, "tcp error: %s (reconnecting)" % e)
            s.close()
            stop.wait(1)
    raw.close()
    idxf.close()


def meas_thread(host, hz, outdir, logf, stats):
    fh = open(os.path.join(outdir, "meas.csv"), "w", newline="")
    w = csv.writer(fh)
    w.writerow(MEAS_COLS)
    period = 1.0 / hz
    while not stop.is_set():
        t = time.time()
        try:
            r = web_cmd(host, "MEAS")
            if r[:2] != b"oS" or r[8:12] != b"SFWA" or len(r) < 60:
                raise ValueError("unexpected reply %s" % r[:16].hex())
            v = struct.unpack_from("<12f", r, 12)
            w.writerow(["%.6f" % t, "%.3f" % v[5], "%.4f" % (v[0] * 3.6), "%.4f" % math.degrees(v[1]),
                        "%.4f" % v[2], "%.4f" % math.degrees(v[3]), "%.4f" % math.degrees(v[4]),
                        "%.2f" % v[6], "%.2f" % v[7], "%.2f" % v[8], "%.0f" % v[9], "%.1f" % v[10], "%.1f" % v[11], r.hex()])
            fh.flush()
            stats["meas"] += 1
            stats["last_meas"] = v
        except Exception as e:
            stats["meas_err"] += 1
            log(logf, "meas error: %s" % e)
        stop.wait(max(0.0, period - (time.time() - t)))
    fh.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.3.50")
    ap.add_argument("--port", type=int, default=51000)
    ap.add_argument("--out", default=os.path.expanduser("~/oms_logs"))
    ap.add_argument("--meas-hz", type=float, default=5.0, help="web MEAS poll rate")
    ap.add_argument("--duration", type=float, default=0, help="seconds, 0 = until SIGINT")
    ap.add_argument("--no-meas", action="store_true", help="raw stream only")
    a = ap.parse_args()

    outdir = os.path.join(a.out, datetime.now().strftime("%Y%m%d_%H%M%S"))
    os.makedirs(outdir)
    logf = open(os.path.join(outdir, "log.txt"), "a")
    log(logf, "recording to %s" % outdir)
    for c in ("INFO", "STAT"):  # one-shot device info for the record
        try:
            log(logf, "%s reply: %s" % (c, web_cmd(a.host, c).hex()))
        except Exception as e:
            log(logf, "%s failed: %s" % (c, e))

    stats: dict = dict(packets=0, frames=0, dropped=0, last_st=None, meas=0, meas_err=0, last_meas=None)
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    threads = [threading.Thread(target=tcp_thread, args=(a.host, a.port, outdir, logf, stats), daemon=True)]
    if not a.no_meas:
        threads.append(threading.Thread(target=meas_thread, args=(a.host, a.meas_hz, outdir, logf, stats), daemon=True))
    for th in threads:
        th.start()

    t_end = time.time() + a.duration if a.duration else None
    while not stop.is_set():
        stop.wait(2)
        if t_end and time.time() >= t_end:
            stop.set()
        line = "pkts=%d frames=%d dropped=%d meas=%d meas_err=%d" % (
            stats["packets"], stats["frames"], stats["dropped"], stats["meas"], stats["meas_err"])
        if stats["last_st"] is not None:
            line += " | tcp t=%.3f" % stats["last_st"]
        m = stats["last_meas"]
        if m:
            line += " | web t=%.3f vel=%.1fkm/h ang=%.2fdeg dist=%.3fm pitch=%.2f roll=%.2f" % (
                m[5], m[0] * 3.6, math.degrees(m[1]), m[2], math.degrees(m[3]), math.degrees(m[4]))
        log(logf, line)
    for th in threads:
        th.join(timeout=5)
    log(logf, "stopped")


if __name__ == "__main__":
    main()
