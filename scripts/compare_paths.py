#!/usr/bin/env python3
"""
Midline vs racing line, from a drivebrain MCAP: where the car drove, how curvy it was, how close it got to cones,
and how well it tracked the line it was given.

    pip install mcap mcap-protobuf-support matplotlib numpy
    python scripts/compare_paths.py recordings/4.mcap [--out DIR] [--a-lat 8] [--a-long 4] [--a-brake 6] [--v-max 15]

Reads (all written by drivebrain's autonomy stack):
    slam/vehicle_pose        car pose in the SLAM map frame -> the driven path, split into laps
    scene/midline_path       midline planner output, every scan
    scene/racing_line_path   racing line window ahead of the car, only while the track is closed
    scene/planned_path       what the controller followed (the cross-track error is measured against this)
    slam_landmarks           SLAM cone map (the last one is used as the track)

A lap ends when the car comes back within LAP_RADIUS_M of where it started, after leaving by LEAVE_RADIUS_M.
A lap counts as a racing-line lap when racing_line_path was published during at least half of its scans.
Poses, cones and paths are all SLAM estimates, so clearances and errors carry SLAM's own error (a few tenths of a metre).
Works on a file that is still open (unindexed), but stop drivebrain first so the file is complete.

Metrics, per lap and then averaged per planner:
    curvature            |1/radius| of the driven path (smoothed over ~1 m); mean and 95th percentile
    curvature-limited    lap time if the car always drove as fast as grip allows: v <= sqrt(a_lat / curvature),
    time (estimate)      with acceleration/braking limits. NOT the time driven: it shows what the path is worth
                         once a speed profile exists (the controller currently holds a constant speed)
    cone clearance       distance from the car's reference point to the nearest cone; `contacts` counts separate
                         visits closer than CONTACT_M (0.81 m: half the car width plus the cone radius)
    cross-track error    distance from the car to the line the controller was following
"""
import argparse
import csv
import math
import struct
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from mcap.stream_reader import StreamReader
from mcap_protobuf.decoder import DecoderFactory

POSE, MIDLINE, RACING, PLANNED, LANDMARKS = "slam/vehicle_pose", "scene/midline_path", "scene/racing_line_path", "scene/planned_path", "slam_landmarks"
LANDMARK_SCENE = "scene/slam_landmarks"
LAP_RADIUS_M = 4.0      # back within this of the start = lap done
LEAVE_RADIUS_M = 15.0   # must first have been this far from the start
MIN_LAP_M = 30.0
CONTACT_M = 0.81        # centre-to-cone distance at which the car body touches a cone
GRID_M = 0.5            # arc-length spacing of the resampled driven path
SMOOTH_SIGMA_M = 1.0
RACING_FRACTION = 0.5   # share of a lap's scans with a racing line to call it a racing-line lap
SAMPLE_EVERY = 8        # plot every Nth planner message, so the plot builds up without being a smear

C_MIDLINE, C_RACING, C_CAR = "#e8730c", "#16a34a", "#374151"
CONE_COLORS = {"BLUE": "#2563eb", "YELLOW": "#eab308", "ORANGE_SMALL": "#f97316", "ORANGE_BIG": "#f97316"}


def varint(b, i):
    v = sh = 0
    while True:
        c = b[i]; i += 1
        v |= (c & 0x7F) << sh; sh += 7
        if not c & 0x80:
            return v, i


def parse_fields(b):
    """Minimal protobuf wire parser -> {field: [values]}; floats as ('f32', x), submessages as bytes"""
    out, i = {}, 0
    while i < len(b):
        key, i = varint(b, i)
        fn, wt = key >> 3, key & 7
        if wt == 0:
            v, i = varint(b, i)
        elif wt == 1:
            v = struct.unpack("<d", b[i:i + 8])[0]; i += 8
        elif wt == 5:
            v = struct.unpack("<f", b[i:i + 4])[0]; i += 4
        elif wt == 2:
            n, i = varint(b, i); v = b[i:i + n]; i += n
        else:
            break
        out.setdefault(fn, []).append(v)
    return out


def decode_pose(data):
    """hytech_msgs.pose: 1 = position {1:x 2:y 3:z}, 2 = orientation {1:x 2:y 3:z 4:w}. The logger writes this
    channel without a schema, so it is decoded by hand."""
    f = parse_fields(data)
    pos = parse_fields(f[1][0]) if 1 in f else {}
    ori = parse_fields(f[2][0]) if 2 in f else {}
    x, y = pos.get(1, [0.0])[0], pos.get(2, [0.0])[0]
    z, w = ori.get(3, [0.0])[0], ori.get(4, [1.0])[0]
    return x, y, 2.0 * math.atan2(z, w)


def read(path):
    wanted = {POSE, MIDLINE, RACING, PLANNED, LANDMARKS, LANDMARK_SCENE}
    schemas, channels, raw = {}, {}, {t: [] for t in wanted}
    decoders = DecoderFactory()
    n = 0
    with open(path, "rb") as f:
        try:
            for rec in StreamReader(f).records:
                kind = type(rec).__name__
                if kind == "Schema":
                    schemas[rec.id] = rec
                elif kind == "Channel":
                    channels[rec.id] = rec
                elif kind == "Message":
                    ch = channels.get(rec.channel_id)
                    if ch is None or ch.topic not in wanted:
                        continue
                    if ch.topic in (LANDMARKS, LANDMARK_SCENE):
                        raw[ch.topic] = [(rec.log_time, ch.schema_id, rec.data)]  # only the last one is used
                    else:
                        raw[ch.topic].append((rec.log_time, ch.schema_id, rec.data))
                    n += 1
        except Exception as e:  # a file that was cut off mid-write still has everything before the cut
            print(f"note: stopped reading at {type(e).__name__}: {e}", file=sys.stderr)

    def decode(topic, items):
        out = []
        cache = {}
        for t, sid, data in items:
            if sid not in cache:
                cache[sid] = decoders.decoder_for("protobuf", schemas[sid]) if sid in schemas else None
            if cache[sid] is not None:
                out.append((t, cache[sid](data)))
        return out

    pose = np.array([(t * 1e-9, *decode_pose(d)) for t, _, d in raw[POSE]]) if raw[POSE] else np.zeros((0, 4))
    lines = {}
    for topic in (MIDLINE, RACING, PLANNED):
        lines[topic] = []
        for t, msg in decode(topic, raw[topic]):
            for ent in msg.entities:
                for ln in ent.lines:
                    if len(ln.points) >= 2:
                        lines[topic].append((t * 1e-9, np.array([[p.x, p.y] for p in ln.points])))
    msg_times = {topic: np.array([t * 1e-9 for t, _, _ in raw[topic]]) for topic in (MIDLINE, RACING, PLANNED)}

    cones = []  # (x, y, colour name)
    if raw[LANDMARKS]:
        for _, msg in decode(LANDMARKS, raw[LANDMARKS]):
            enum = msg.DESCRIPTOR.fields_by_name["cones"].message_type.fields_by_name["color"].enum_type
            cones = [(c.position.x, c.position.y, enum.values_by_number[c.color].name) for c in msg.cones]
    elif raw[LANDMARK_SCENE]:
        for _, msg in decode(LANDMARK_SCENE, raw[LANDMARK_SCENE]):
            for ent in msg.entities:
                for cyl in ent.cylinders:
                    c = cyl.color
                    name = "BLUE" if c.b > c.r else ("YELLOW" if c.g > 0.6 else "ORANGE_SMALL")
                    cones.append((cyl.pose.position.x, cyl.pose.position.y, name))
    return pose, lines, msg_times, cones


def find_laps(pose):
    """-> list of dicts for every completed lap, plus the unfinished tail"""
    laps, p0 = [], pose[0, 1:3]
    start_i, left, dist = 0, False, 0.0
    for i in range(1, len(pose)):
        dist += float(np.hypot(*(pose[i, 1:3] - pose[i - 1, 1:3])))
        d0 = float(np.hypot(*(pose[i, 1:3] - p0)))
        if d0 > LEAVE_RADIUS_M:
            left = True
        if left and d0 < LAP_RADIUS_M and dist > MIN_LAP_M:
            laps.append(dict(start=pose[start_i, 0], end=pose[i, 0], distance=dist, first=start_i, last=i))
            start_i, left, dist = i, False, 0.0
    tail = dict(start=pose[start_i, 0], end=pose[-1, 0], distance=dist, first=start_i, last=len(pose) - 1)
    return laps, tail


def smooth(v, sigma_samples):
    half = int(3 * sigma_samples)
    k = np.exp(-0.5 * (np.arange(-half, half + 1) / sigma_samples) ** 2)
    k /= k.sum()
    return np.convolve(np.pad(v, half, mode="edge"), k, mode="valid")


def driven_curvature(xy):
    """Resample a driven path by arc length and return (s, x, y, |curvature|) on a GRID_M grid"""
    step = np.hypot(*np.diff(xy, axis=0).T)
    keep = np.concatenate([[True], step > 1e-3])
    xy = xy[keep]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    g = np.arange(0.0, s[-1], GRID_M)
    x = smooth(np.interp(g, s, xy[:, 0]), SMOOTH_SIGMA_M / GRID_M)
    y = smooth(np.interp(g, s, xy[:, 1]), SMOOTH_SIGMA_M / GRID_M)
    dx, dy = np.gradient(x, GRID_M), np.gradient(y, GRID_M)
    ddx, ddy = np.gradient(dx, GRID_M), np.gradient(dy, GRID_M)
    kappa = np.abs(dx * ddy - dy * ddx) / np.maximum(dx * dx + dy * dy, 1e-9) ** 1.5
    return g, x, y, kappa


def speed_limited_time(kappa, a_lat, a_long, a_brake, v_max):
    """Time to drive the path as fast as grip allows. Closed loop: the speed profile wraps around."""
    n = len(kappa)
    v = np.minimum(v_max, np.sqrt(a_lat / np.maximum(kappa, 1e-3)))
    for _ in range(2):  # two sweeps so the profile is consistent across the start/finish line
        for i in range(2 * n):
            j, k = i % n, (i + 1) % n
            v[k] = min(v[k], math.sqrt(v[j] ** 2 + 2 * a_long * GRID_M))
        for i in range(2 * n, 0, -1):
            j, k = i % n, (i - 1) % n
            v[k] = min(v[k], math.sqrt(v[j] ** 2 + 2 * a_brake * GRID_M))
    return float(np.sum(GRID_M / np.maximum(v, 0.5)))


def dist_to_polyline(p, pts):
    """Sideways distance from p to a path. The path may start ahead of the car (the midline planner starts at the first
    midpoint in front of it), so the first segment is treated as extending backwards without limit: what is measured is
    how far to the side of the line the car is, not how far it is from the start of the path."""
    a, b = pts[:-1], pts[1:]
    ab = b - a
    t = np.einsum("ij,ij->i", p - a, ab) / np.maximum(np.einsum("ij,ij->i", ab, ab), 1e-9)
    t[1:] = np.clip(t[1:], 0.0, 1.0)
    t[0] = min(t[0], 1.0)
    return float(np.min(np.hypot(*(a + t[:, None] * ab - p).T)))


def count_events(mask):
    return int(np.sum(mask[1:] & ~mask[:-1]) + (1 if mask[0] else 0)) if len(mask) else 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mcap")
    ap.add_argument("--out", default=".", help="directory for the PNG and CSV (default: current directory)")
    ap.add_argument("--a-lat", type=float, default=8.0, help="lateral acceleration limit, m/s^2 (placeholder, set from the real car)")
    ap.add_argument("--a-long", type=float, default=4.0, help="acceleration limit, m/s^2")
    ap.add_argument("--a-brake", type=float, default=6.0, help="braking limit, m/s^2")
    ap.add_argument("--v-max", type=float, default=15.0, help="top speed, m/s")
    args = ap.parse_args()

    print(f"reading {args.mcap} (a large file takes a minute or two) ...")
    pose, lines, times, cones = read(args.mcap)
    if len(pose) < 10:
        sys.exit(f"no '{POSE}' messages in this file, so there is no car path to analyse")
    print(f"{len(pose)} poses over {pose[-1, 0] - pose[0, 0]:.0f} s, {len(lines[MIDLINE])} midline / {len(lines[RACING])} racing-line messages")

    laps, tail = find_laps(pose)
    if not laps:
        sys.exit("no complete lap in this file (the car never came back to its start)")
    cone_xy = np.array([(x, y) for x, y, _ in cones]) if cones else np.zeros((0, 2))
    planned = lines[PLANNED] or lines[MIDLINE]
    planned_t = np.array([t for t, _ in planned])

    for k, lap in enumerate(laps, 1):
        lap["n"] = k
        in_lap = lambda tt: tt[(tt >= lap["start"]) & (tt < lap["end"])]
        scans = len(in_lap(times[PLANNED])) or len(in_lap(times[MIDLINE]))
        lap["racing_fraction"] = len(in_lap(times[RACING])) / scans if scans else 0.0
        lap["planner"] = "racing line" if lap["racing_fraction"] >= RACING_FRACTION else "midline"

        sl = slice(lap["first"], lap["last"] + 1)
        xy, t = pose[sl, 1:3], pose[sl, 0]
        cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
        lap["u_pose"] = cum / max(cum[-1], 1e-9)

        g, x, y, kappa = driven_curvature(xy)
        lap["u_kappa"], lap["kappa"] = g / g[-1], kappa
        lap["kappa_mean"], lap["kappa_p95"] = float(np.mean(kappa)), float(np.percentile(kappa, 95))
        lap["est_time"] = speed_limited_time(kappa, args.a_lat, args.a_long, args.a_brake, args.v_max)

        if len(cone_xy):
            lap["clear"] = np.min(np.hypot(xy[:, None, 0] - cone_xy[None, :, 0], xy[:, None, 1] - cone_xy[None, :, 1]), axis=1)
            lap["clear_min"] = float(lap["clear"].min())
            lap["contacts"] = count_events(lap["clear"] < CONTACT_M)
        else:
            lap["clear"], lap["clear_min"], lap["contacts"] = np.full(len(xy), np.nan), float("nan"), 0

        cte_u, cte = [], []
        for (tp, pts) in planned[np.searchsorted(planned_t, lap["start"]):np.searchsorted(planned_t, lap["end"])]:
            px, py = np.interp(tp, t, xy[:, 0]), np.interp(tp, t, xy[:, 1])
            cte.append(dist_to_polyline(np.array([px, py]), pts))
            cte_u.append(np.interp(tp, t, lap["u_pose"]))
        lap["u_cte"], lap["cte"] = np.array(cte_u), np.array(cte)
        lap["cte_mean"] = float(np.mean(cte)) if cte else float("nan")
        lap["cte_p95"] = float(np.percentile(cte, 95)) if cte else float("nan")

    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    stem = Path(args.mcap).stem
    cols = [("lap", "{:>3d}", "n"), ("planner", "{:<11s}", "planner"), ("dist m", "{:6.0f}", "distance"),
            ("curv mean", "{:9.3f}", "kappa_mean"), ("curv p95", "{:8.3f}", "kappa_p95"), ("est. s", "{:6.1f}", "est_time"),
            ("clear min", "{:9.2f}", "clear_min"), ("contacts", "{:8d}", "contacts"),
            ("cte mean", "{:8.2f}", "cte_mean"), ("cte p95", "{:7.2f}", "cte_p95")]
    print("\n" + " ".join(f"{c[0]:>{len(c[1].format(*([0] if 'd' in c[1] else [0.0] if 'f' in c[1] else ['x'])))}}" for c in cols))
    for lap in laps:
        print(" ".join(c[1].format(lap[c[2]]) for c in cols))
    with open(out / f"{stem}_metrics.csv", "w", newline="") as fh:
        w = csv.writer(fh); w.writerow([c[0] for c in cols] + ["racing_fraction"])
        for lap in laps:
            w.writerow([lap[c[2]] for c in cols] + [f"{lap['racing_fraction']:.3f}"])

    summary = []
    groups = {name: [l for l in laps if l["planner"] == name] for name in ("midline", "racing line")}
    print(f"\nper planner (curvature-limited time uses a_lat={args.a_lat}, a_long={args.a_long}, a_brake={args.a_brake}, v_max={args.v_max}):")
    for name, ls in groups.items():
        if not ls:
            continue
        line = (f"{name:11s} {len(ls)} lap(s): curvature mean {np.mean([l['kappa_mean'] for l in ls]):.3f} p95 {np.mean([l['kappa_p95'] for l in ls]):.3f} 1/m, "
                f"est. lap {np.mean([l['est_time'] for l in ls]):.1f} s, closest cone {min(l['clear_min'] for l in ls):.2f} m, "
                f"contacts {sum(l['contacts'] for l in ls)}, cross-track mean {np.nanmean([l['cte_mean'] for l in ls]):.2f} m p95 {np.nanmean([l['cte_p95'] for l in ls]):.2f} m")
        summary.append(line); print(line)
    if all(groups.values()):
        m, r = groups["midline"], groups["racing line"]
        em, er = np.mean([l["est_time"] for l in m]), np.mean([l["est_time"] for l in r])
        km, kr = np.mean([l["kappa_p95"] for l in m]), np.mean([l["kappa_p95"] for l in r])
        d = f"racing line vs midline: curvature p95 {100 * (kr - km) / km:+.0f}%, curvature-limited lap {er - em:+.1f} s ({100 * (er - em) / em:+.1f}%)"
        summary.append(d); print(d)

    fig = plt.figure(figsize=(17, 9.5))
    gs = fig.add_gridspec(3, 2, width_ratios=[1.5, 1])
    ax = fig.add_subplot(gs[:, 0])
    for x, y, name in cones:
        ax.scatter(x, y, s=14, color=CONE_COLORS.get(name, "#9ca3af"), zorder=3)
    ax.plot(pose[:, 1], pose[:, 2], color=C_CAR, lw=1.0, alpha=0.55, zorder=2)
    for topic, color in ((MIDLINE, C_MIDLINE), (RACING, C_RACING)):
        for _, pts in lines[topic][::SAMPLE_EVERY]:
            ax.plot(pts[:, 0], pts[:, 1], color=color, lw=1.4, alpha=0.35, zorder=4 if topic == RACING else 3)
    ax.set_aspect("equal"); ax.grid(alpha=0.2)
    ax.set_title(f"{Path(args.mcap).name}: track, car path and planned lines (SLAM map frame)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.legend(handles=[plt.Line2D([], [], color=C_CAR, lw=1.5, alpha=0.7, label="car path (SLAM)"),
                       plt.Line2D([], [], color=C_MIDLINE, lw=2, label="midline planner"),
                       plt.Line2D([], [], color=C_RACING, lw=2, label="racing line"),
                       plt.Line2D([], [], marker="o", ls="", color=CONE_COLORS["BLUE"], label="blue cone"),
                       plt.Line2D([], [], marker="o", ls="", color=CONE_COLORS["YELLOW"], label="yellow cone")], loc="best")

    color_of = lambda lap: C_RACING if lap["planner"] == "racing line" else C_MIDLINE
    a1, a2, a3 = (fig.add_subplot(gs[i, 1]) for i in range(3))
    for lap in laps:
        c = color_of(lap)
        a1.plot(lap["u_kappa"], lap["kappa"], color=c, lw=1.0, alpha=0.8)
        a2.plot(lap["u_pose"], lap["clear"], color=c, lw=1.0, alpha=0.8)
        a3.plot(lap["u_cte"], lap["cte"], color=c, lw=0.8, alpha=0.8)
    a1.set_ylabel("curvature (1/m)"); a1.set_title("curvature of the driven path along the lap")
    a2.axhline(CONTACT_M, color="#dc2626", ls="--", lw=1); a2.text(0.01, CONTACT_M, " cone contact", color="#dc2626", va="bottom", fontsize=8)
    a2.set_ylabel("nearest cone (m)"); a2.set_title("clearance to the nearest cone"); a2.set_ylim(bottom=0)
    a3.set_ylabel("cross-track error (m)"); a3.set_title("distance from the line the controller was following"); a3.set_xlabel("fraction of the lap")
    for a in (a1, a2, a3):
        a.grid(alpha=0.2)
    a1.legend(handles=[plt.Line2D([], [], color=C_MIDLINE, label="midline laps"), plt.Line2D([], [], color=C_RACING, label="racing-line laps")], loc="upper right")
    fig.suptitle("\n".join(summary), fontsize=9, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    png = out / f"{stem}_paths.png"
    fig.savefig(png, dpi=125)
    print(f"\nwrote {png}\nwrote {out / (stem + '_metrics.csv')}")


if __name__ == "__main__":
    main()
