#!/usr/bin/env python3
"""
Midline vs racing line, from a drivebrain MCAP: where the car drove, how curvy it was, how close it got to cones,
and how well it tracked the line it was given.

    pip install mcap mcap-protobuf-support matplotlib numpy
    python scripts/compare_paths.py recordings/4.mcap [--out DIR] [--discovery-laps 1] [--a-lat 8] [--a-long 4] [--a-brake 6] [--v-max 15]

Reads (all written by drivebrain's autonomy stack):
    slam/vehicle_pose        car pose in the SLAM map frame -> the driven path, split into laps
    scene/midline_path       midline planner output, every scan
    scene/racing_line_path   racing line window ahead of the car, only while the track is closed
    scene/planned_path       what the controller followed (the cross-track error is measured against this)
    slam_landmarks           SLAM cone map (the last one is used as the track)

A lap ends when the car comes back within LAP_RADIUS_M of where it started, after leaving by LEAVE_RADIUS_M.
The intended run is three laps, with racing_line/midline_laps_after_closure = 1:
    lap 1  "discovery"    midline planner while SLAM maps the track (the map is incomplete, so it is not a fair baseline)
    lap 2  "midline"      midline planner on the closed map: the baseline
    lap 3  "racing line"  racing line on the same map
A lap is a racing-line lap when racing_line_path was published during at least half of its scans (the planner only
publishes it once the midline hold is over). The first --discovery-laps laps that are not racing-line laps are
"discovery" and left out of the comparison; every other lap is "midline". The summary compares midline with racing line.
Laps after the third are averaged in as more racing-line laps. An unfinished last lap is reported but not scored.
Poses, cones and paths are all SLAM estimates, so clearances and errors carry SLAM's own error (a few tenths of a metre).
Works on a file that is still open (unindexed), but stop drivebrain first so the file is complete.

Metrics, per lap and then averaged per planner:
    curvature            |1/radius| of the driven path (smoothed over ~1 m); mean and 95th percentile
    curvature rate       |d curvature / d distance| along the path (1/m^2), mean and 95th percentile: how fast the
                         steering would have to change. A jittery path scores high even if its curvature is low
    curvature-limited    lap time if the car always drove as fast as grip allows: v <= sqrt(a_lat / curvature),
    time (estimate)      with acceleration/braking limits (the speed profile is plotted). NOT the time driven: it
                         shows what the path is worth once a speed profile exists (the controller currently holds a
                         constant speed). The a_lat/a_long/a_brake defaults are placeholders: set them from the car
    cone clearance       distance from the car's reference point to the nearest cone; `contacts` counts separate
                         visits closer than CONTACT_M (0.81 m: half the car width plus the cone radius)
    body gap             gap between the car's rectangular outline and the nearest cone's edge, using the pose heading
                         (so the corners count). `body hits` counts separate visits with a negative gap. The outline is
                         an assumption: --car-half-width, --car-front and --car-rear measured from the pose reference point
    cross-track error    sideways distance from the car to the line the controller was following (scene/planned_path),
                         mean and 95th percentile: how closely it tracked its path
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

C_MIDLINE, C_RACING, C_DISCOVERY = "#e8730c", "#16a34a", "#6b7280"
LAP_COLORS = {"discovery": C_DISCOVERY, "midline": C_MIDLINE, "racing line": C_RACING}
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
    """Resample a driven path by arc length and return (s, x, y, signed curvature) on a GRID_M grid"""
    step = np.hypot(*np.diff(xy, axis=0).T)
    keep = np.concatenate([[True], step > 1e-3])
    xy = xy[keep]
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    g = np.arange(0.0, s[-1], GRID_M)
    x = smooth(np.interp(g, s, xy[:, 0]), SMOOTH_SIGMA_M / GRID_M)
    y = smooth(np.interp(g, s, xy[:, 1]), SMOOTH_SIGMA_M / GRID_M)
    dx, dy = np.gradient(x, GRID_M), np.gradient(y, GRID_M)
    ddx, ddy = np.gradient(dx, GRID_M), np.gradient(dy, GRID_M)
    kappa = (dx * ddy - dy * ddx) / np.maximum(dx * dx + dy * dy, 1e-9) ** 1.5  # signed: left turns positive
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
    return float(np.sum(GRID_M / np.maximum(v, 0.5))), v


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
    ap.add_argument("--discovery-laps", type=int, default=1, help="leading laps driven while the track was still being mapped, left out of the comparison")
    ap.add_argument("--car-half-width", type=float, default=0.70, help="half the car's width, m (same as racing_line/car_half_width_m)")
    ap.add_argument("--car-front", type=float, default=1.4, help="pose reference point to the front of the body, m (assumed)")
    ap.add_argument("--car-rear", type=float, default=1.4, help="pose reference point to the rear of the body, m (assumed)")
    ap.add_argument("--cone-radius", type=float, default=0.114, help="cone radius, m")
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
        lap["lap_time"] = lap["end"] - lap["start"]  # the time actually driven
        in_lap = lambda tt: tt[(tt >= lap["start"]) & (tt < lap["end"])]
        scans = len(in_lap(times[PLANNED])) or len(in_lap(times[MIDLINE]))
        lap["racing_fraction"] = len(in_lap(times[RACING])) / scans if scans else 0.0
        lap["racing_pct"] = 100.0 * lap["racing_fraction"]
        if lap["racing_fraction"] >= RACING_FRACTION:
            lap["planner"] = "racing line"
        elif k <= args.discovery_laps:
            lap["planner"] = "discovery"
        else:
            lap["planner"] = "midline"

        sl = slice(lap["first"], lap["last"] + 1)
        xy, t = pose[sl, 1:3], pose[sl, 0]
        cum = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
        lap["u_pose"] = cum / max(cum[-1], 1e-9)

        g, x, y, kappa_signed = driven_curvature(xy)
        kappa = np.abs(kappa_signed)
        lap["u_kappa"], lap["kappa"] = g / g[-1], kappa
        lap["kappa_mean"], lap["kappa_p95"] = float(np.mean(kappa)), float(np.percentile(kappa, 95))
        rate = np.abs(smooth(np.gradient(kappa_signed, GRID_M), SMOOTH_SIGMA_M / GRID_M))
        lap["rate"] = rate
        lap["rate_mean"], lap["rate_p95"] = float(np.mean(rate)), float(np.percentile(rate, 95))
        lap["est_time"], lap["speed"] = speed_limited_time(kappa, args.a_lat, args.a_long, args.a_brake, args.v_max)

        if len(cone_xy):
            lap["clear"] = np.min(np.hypot(xy[:, None, 0] - cone_xy[None, :, 0], xy[:, None, 1] - cone_xy[None, :, 1]), axis=1)
            lap["clear_min"] = float(lap["clear"].min())
            lap["contacts"] = count_events(lap["clear"] < CONTACT_M)
            # rectangle footprint: cones into the car frame, distance to the rectangle, minus the cone radius
            yaw = pose[sl, 3]
            c, sn = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
            dx, dy = cone_xy[None, :, 0] - xy[:, None, 0], cone_xy[None, :, 1] - xy[:, None, 1]
            fwd, left = c * dx + sn * dy, -sn * dx + c * dy
            du = np.maximum(np.maximum(-args.car_rear - fwd, fwd - args.car_front), 0.0)
            dv = np.maximum(np.abs(left) - args.car_half_width, 0.0)
            lap["body_gap"] = np.min(np.hypot(du, dv), axis=1) - args.cone_radius
            lap["body_min"] = float(lap["body_gap"].min())
            lap["body_hits"] = count_events(lap["body_gap"] < 0.0)
        else:
            lap["clear"], lap["clear_min"], lap["contacts"] = np.full(len(xy), np.nan), float("nan"), 0
            lap["body_gap"], lap["body_min"], lap["body_hits"] = np.full(len(xy), np.nan), float("nan"), 0

        # tracking: how far to the side of the line the controller was following (planned_path) the car was, at each scan
        cte = []
        for (tp, pts) in planned[np.searchsorted(planned_t, lap["start"]):np.searchsorted(planned_t, lap["end"])]:
            cte.append(dist_to_polyline(np.array([np.interp(tp, t, xy[:, 0]), np.interp(tp, t, xy[:, 1])]), pts))
        lap["cte_mean"] = float(np.mean(cte)) if cte else float("nan")
        lap["cte_p95"] = float(np.percentile(cte, 95)) if cte else float("nan")


    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    stem = Path(args.mcap).stem
    cols = [("lap", "{:>3d}", "n"), ("planner", "{:<11s}", "planner"), ("lap s", "{:6.1f}", "lap_time"), ("dist m", "{:6.0f}", "distance"),
            ("curv mean", "{:9.3f}", "kappa_mean"), ("curv p95", "{:8.3f}", "kappa_p95"), ("rate mean", "{:9.4f}", "rate_mean"), ("rate p95", "{:8.4f}", "rate_p95"), ("est. s", "{:6.1f}", "est_time"),
            ("clear min", "{:9.2f}", "clear_min"), ("contacts", "{:8d}", "contacts"),
            ("body gap", "{:8.2f}", "body_min"), ("body hits", "{:9d}", "body_hits"),
            ("cte mean", "{:8.2f}", "cte_mean"), ("cte p95", "{:7.2f}", "cte_p95"),
            ("racing %", "{:8.0f}", "racing_pct")]
    print("\n" + " ".join(f"{c[0]:>{len(c[1].format(*([0] if 'd' in c[1] else [0.0] if 'f' in c[1] else ['x'])))}}" for c in cols))
    for lap in laps:
        print(" ".join(c[1].format(lap[c[2]]) for c in cols))
    with open(out / f"{stem}_metrics.csv", "w", newline="") as fh:
        w = csv.writer(fh); w.writerow([c[0] for c in cols] + ["racing_fraction"])
        for lap in laps:
            w.writerow([lap[c[2]] for c in cols] + [f"{lap['racing_fraction']:.3f}"])

    header = []
    groups = {name: [l for l in laps if l["planner"] == name] for name in ("midline", "racing line")}
    if tail["distance"] > 10.0:
        print(f"unfinished lap after the last complete one: {tail['distance']:.0f} m driven, not scored")
    for name in ("midline", "racing line"):
        if not groups[name]:
            print(f"no complete {name} lap: the comparison needs one midline lap after the discovery lap and one racing-line lap")
    print(f"\nper planner (curvature-limited time uses a_lat={args.a_lat}, a_long={args.a_long}, a_brake={args.a_brake}, v_max={args.v_max}):")
    for name, ls in groups.items():
        if not ls:
            continue
        line = (f"{name:11s} {len(ls)} lap(s): curvature mean {np.mean([l['kappa_mean'] for l in ls]):.3f} p95 {np.mean([l['kappa_p95'] for l in ls]):.3f} 1/m, "
                f"curvature rate mean {np.mean([l['rate_mean'] for l in ls]):.4f} p95 {np.mean([l['rate_p95'] for l in ls]):.4f} 1/m^2, "
                f"est. lap {np.mean([l['est_time'] for l in ls]):.1f} s, closest cone {min(l['clear_min'] for l in ls):.2f} m, "
                f"contacts {sum(l['contacts'] for l in ls)}, body gap min {min(l['body_min'] for l in ls):.2f} m (hits {sum(l['body_hits'] for l in ls)})")
        print(line)
        header.append(f"{name.capitalize()} (lap {', '.join(str(l['n']) for l in ls)}): {np.mean([l['lap_time'] for l in ls]):.1f} s, "
                      f"curvature p95 {np.mean([l['kappa_p95'] for l in ls]):.3f} 1/m, curvature rate p95 {np.mean([l['rate_p95'] for l in ls]):.4f} 1/m², "
                      f"body gap {min(l['body_min'] for l in ls):.2f} m")
    if all(groups.values()):
        m, r = groups["midline"], groups["racing line"]
        em, er = np.mean([l["est_time"] for l in m]), np.mean([l["est_time"] for l in r])
        km, kr = np.mean([l["kappa_p95"] for l in m]), np.mean([l["kappa_p95"] for l in r])
        rm, rr = np.mean([l["rate_p95"] for l in m]), np.mean([l["rate_p95"] for l in r])
        d = (f"racing line vs midline: curvature p95 {100 * (kr - km) / km:+.0f}%, curvature rate p95 {100 * (rr - rm) / rm:+.0f}%, "
             f"curvature-limited lap {er - em:+.1f} s ({100 * (er - em) / em:+.1f}%)")
        print(d)

    fig = plt.figure(figsize=(17, 12))
    gs = fig.add_gridspec(4, 2, width_ratios=[1.5, 1])
    ax = fig.add_subplot(gs[:, 0])
    for x, y, name in cones:
        ax.scatter(x, y, s=14, color=CONE_COLORS.get(name, "#9ca3af"), zorder=3)
    for lap in laps:  # the driven path of each lap, in its planner's colour
        sl = slice(lap["first"], lap["last"] + 1)
        discovery = lap["planner"] == "discovery"  # dashed and on top, so it stays visible where the later laps overlap it
        ax.plot(pose[sl, 1], pose[sl, 2], color=LAP_COLORS[lap["planner"]], lw=1.6 if discovery else 1.8, ls="--" if discovery else "-",
                zorder=6 if discovery else 5)
    ax.set_aspect("equal"); ax.grid(alpha=0.2)
    ax.set_title(f"{Path(args.mcap).name}: driven path by lap (SLAM map frame)")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.legend(handles=[plt.Line2D([], [], color=LAP_COLORS[l["planner"]], lw=2, ls="--" if l["planner"] == "discovery" else "-", label=f"Lap {l['n']} ({l['planner']})") for l in laps] + [
                       plt.Line2D([], [], marker="o", ls="", color=CONE_COLORS["BLUE"], label="blue cone"),
                       plt.Line2D([], [], marker="o", ls="", color=CONE_COLORS["YELLOW"], label="yellow cone")], loc="best")

    color_of = lambda lap: LAP_COLORS[lap["planner"]]
    a1, a1r, a2 = (fig.add_subplot(gs[i, 1]) for i in (0, 1, 2))
    for lap in laps:
        c = color_of(lap)
        a1.plot(lap["u_kappa"], lap["kappa"], color=c, lw=1.0, alpha=0.8)
        a1r.plot(lap["u_kappa"], lap["rate"], color=c, lw=1.0, alpha=0.8)
        a2.plot(lap["u_pose"], lap["clear"], color=c, lw=1.0, alpha=0.8)
    a1.set_ylabel("Curvature (1/m)"); a1.set_title("Curvature of the driven path along the lap")
    a2.axhline(CONTACT_M, color="#dc2626", ls="--", lw=1); a2.text(0.01, CONTACT_M, " cone contact", color="#dc2626", va="bottom", fontsize=8)
    a2.set_ylabel("nearest cone (m)"); a2.set_title("clearance to the nearest cone"); a2.set_ylim(bottom=0)
    a1r.set_ylabel("Curvature rate (1/m²)"); a1r.set_title("Rate of change of path curvature along the lap")
    a2.set_xlabel("fraction of the lap")
    for a in (a1, a1r, a2):
        a.grid(alpha=0.2)
    a1.legend(handles=[plt.Line2D([], [], color=C_DISCOVERY, label="discovery lap"), plt.Line2D([], [], color=C_MIDLINE, label="midline laps"), plt.Line2D([], [], color=C_RACING, label="racing-line laps")], loc="upper right")
    at = fig.add_subplot(gs[3, 1])  # lap table: what was actually driven
    at.axis("off")
    rows = [[str(l["n"]), l["planner"], f"{l['lap_time']:.1f} s", f"{l['distance']:.0f} m", f"{l['cte_mean']:.2f} / {l['cte_p95']:.2f} m"] for l in laps]
    tbl = at.table(cellText=rows, colLabels=["Lap", "Planner", "Time", "Distance", "Cross-track (mean / p95)"], cellLoc="left", colLoc="left",
                   loc="center", colWidths=[0.08, 0.2, 0.16, 0.18, 0.38])
    tbl.auto_set_font_size(False); tbl.set_fontsize(10); tbl.scale(1, 1.8)
    for (r, _), cell in tbl.get_celld().items():
        cell.set_edgecolor("#d1d5db")
        if r == 0:
            cell.set_text_props(weight="bold"); cell.set_facecolor("#f3f4f6")
    fig.suptitle("\n".join(header), fontsize=10, y=0.995)
    fig.tight_layout(rect=(0, 0, 1, 0.96))
    png = out / f"{stem}_paths.png"
    fig.savefig(png, dpi=125)
    print(f"\nwrote {png}\nwrote {out / (stem + '_metrics.csv')}")


if __name__ == "__main__":
    main()
