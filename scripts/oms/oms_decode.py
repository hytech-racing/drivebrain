#!/usr/bin/env python3
"""Decode an OMS Race recording made by oms_record.py and match raw frame words
to the labelled web values. Stdlib only.

    python3 -I oms_decode.py <logdir>              # summary + field matching, writes joined.csv
    python3 -I oms_decode.py <logdir> --frames-csv --stride 10   # also dump every 10th frame

Also accepts a bare raw .bin (summary only).
"""
import argparse
import csv
import math
import os
import struct
from bisect import bisect_left

MAGIC = b"eS\x01\x00"
HDR = struct.Struct("<HHIIHH")
NW = 36

# tentative names in channel-list order; tail per the DBC Status layout
NAMES = ["SampleTime", "VelXPoi", "VelYPoi", "VelAPoi", "AngSPoi", "DistAPoi?", "RadiusPoi", "AccCPoi",
         "Roll", "Pitch", "AccXHor", "AccYHor", "AccZHor", "RateXHor", "RateYHor", "RateZHor",
         "VelX", "VelY", "VelA", "AngS", "DistA?", "Radius", "AccC",
         "AccX", "AccY", "AccZ", "RateX", "RateY", "RateZ",
         "VelASp", "AngSSp", "VelXSp?", "VelYSp?", "w33?", "TempSensor", "Status"]
U32_WORDS = (5, 20, 33, 35)
STATUS_BITS = {0: "Standstill", 1: "DirectionNeg", 2: "RateAdjustActive", 4: "Trigger", 8: "IlluOn",
               9: "IlluOk", 10: "SensorConnected", 12: "AdjustmentActive", 13: "CanAngAdjFailed", 15: "StatusOk"}


def load_raw(path):
    """Return a list of (floats[36], u32[36]) frames, resyncing on the packet magic."""
    d = open(path, "rb").read()
    frames, pos = [], 0
    while True:
        pos = d.find(MAGIC, pos)
        if pos < 0 or pos + HDR.size > len(d):
            break
        _, _, _, ln, _, fs = HDR.unpack_from(d, pos)
        if pos + 12 + ln > len(d):
            break
        for k in range((ln - 4) // fs):
            off = pos + 16 + k * fs
            frames.append((struct.unpack_from("<36f", d, off), struct.unpack_from("<36I", d, off)))
        pos += 12 + ln
    return frames


def summary(frames):
    st = [f[0][0] for f in frames]
    expected = round((st[-1] - st[0]) * 1000) + 1
    print("frames %d  SampleTime %.3f -> %.3f  (%.1f s, %d dropped)" % (len(frames), st[0], st[-1], st[-1] - st[0], expected - len(frames)))
    print("%3s %-12s %12s %12s %12s %12s   u32 (if few distinct)" % ("w", "name", "first", "min", "max", "last"))
    for i in range(NW):
        col = [f[0][i] for f in frames]
        distinct = sorted(set(f[1][i] for f in frames))
        extra = " ".join("0x%x" % v for v in distinct[:6]) if len(distinct) <= 6 else "(%d distinct)" % len(distinct)
        print("%3d %-12s %12.5g %12.5g %12.5g %12.5g   %s" % (i, NAMES[i], col[0], min(col), max(col), col[-1], extra))
    s = frames[-1][1][35]
    print("last Status 0x%08x: %s" % (s, ", ".join(STATUS_BITS.get(b, "bit%d" % b) for b in range(16) if s >> b & 1)))


def pearson(x, y):
    n = len(x)
    mx, my = sum(x) / n, sum(y) / n
    sxx = sum((a - mx) ** 2 for a in x)
    syy = sum((b - my) ** 2 for b in y)
    if sxx == 0 or syy == 0:
        return 0.0, 0.0
    sxy = sum((a - mx) * (b - my) for a, b in zip(x, y))
    return sxy / math.sqrt(sxx * syy), sxy / sxx  # r, slope


def join_and_match(frames, meas_path, outdir):
    rows = list(csv.DictReader(open(meas_path, newline="")))
    st = [f[0][0] for f in frames]
    joined = []
    for r in rows:
        t = float(r["sampletime"])
        i = bisect_left(st, t)
        cands = [j for j in (i - 1, i) if 0 <= j < len(st)]
        j = min(cands, key=lambda k: abs(st[k] - t))
        if abs(st[j] - t) <= 0.002:
            joined.append((r, frames[j]))
    print("\nweb samples %d, matched to a raw frame within 2 ms: %d" % (len(rows), len(joined)))
    if len(joined) < 10:
        return
    with open(os.path.join(outdir, "joined.csv"), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow([c for c in rows[0] if c != "raw_hex"] + ["w%02d_%s" % (i, NAMES[i]) for i in range(NW)])
        for r, (fl, u) in joined:
            w.writerow([r[c] for c in r if c != "raw_hex"] + [u[i] if i in U32_WORDS else "%.6g" % fl[i] for i in range(NW)])
    print("wrote joined.csv")

    # candidate web quantities in every plausible unit vs every raw word
    web = {
        "vel km/h": [float(r["vel_kmh"]) for r, _ in joined],
        "vel m/s": [float(r["vel_kmh"]) / 3.6 for r, _ in joined],
        "angle deg": [float(r["angle_deg"]) for r, _ in joined],
        "angle rad": [math.radians(float(r["angle_deg"])) for r, _ in joined],
        "dist m": [float(r["dist_m"]) for r, _ in joined],
        "dist mm": [float(r["dist_m"]) * 1000 for r, _ in joined],
        "pitch deg": [float(r["pitch_deg"]) for r, _ in joined],
        "roll deg": [float(r["roll_deg"]) for r, _ in joined],
        "temp_sensor": [float(r["temp_sensor"]) for r, _ in joined],
    }
    print("\nbest raw-word matches per web value (|r| > 0.9, slope ~1 means same unit):")
    for name, y in web.items():
        hits = []
        for i in range(NW):
            x = [u[i] if i in U32_WORDS else fl[i] for _, (fl, u) in joined]
            r, slope = pearson(x, y)
            if abs(r) > 0.9:
                hits.append((abs(r), i, r, slope))
        hits.sort(reverse=True)
        print("  %-12s " % name + ("; ".join("w%02d %s r=%.3f slope=%.4g" % (i, NAMES[i], r, s) for _, i, r, s in hits[:4]) or "-"))

    # internal consistency checks that need no web data
    fl = [f[0] for _, f in joined]
    va = [math.hypot(f[16], f[17]) for f in fl]
    print("\nphysics: r(w18, hypot(w16,w17)) = %.3f   r(w19, atan2d(w17,w16)) = %.3f   r(w03, hypot(w01,w02)) = %.3f" % (
        pearson([f[18] for f in fl], va)[0],
        pearson([f[19] for f in fl], [math.degrees(math.atan2(f[17], f[16])) for f in fl])[0],
        pearson([f[3] for f in fl], [math.hypot(f[1], f[2]) for f in fl])[0]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", help="log directory from oms_record.py, or a raw .bin")
    ap.add_argument("--frames-csv", action="store_true")
    ap.add_argument("--stride", type=int, default=1)
    a = ap.parse_args()

    raw = a.path if a.path.endswith(".bin") else os.path.join(a.path, "raw_51000.bin")
    frames = load_raw(raw)
    if not frames:
        print("no frames found")
        return
    summary(frames)
    outdir = os.path.dirname(raw) or "."
    if a.frames_csv:
        with open(os.path.join(outdir, "frames.csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["w%02d_%s" % (i, NAMES[i]) for i in range(NW)])
            for fl, u in frames[::a.stride]:
                w.writerow([u[i] if i in U32_WORDS else "%.6g" % fl[i] for i in range(NW)])
        print("wrote frames.csv (stride %d)" % a.stride)
    meas = os.path.join(outdir, "meas.csv")
    if os.path.exists(meas):
        join_and_match(frames, meas, outdir)


if __name__ == "__main__":
    main()
