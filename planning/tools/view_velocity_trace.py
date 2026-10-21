"""Interactively inspect VelocityPlanner JSONL traces, including a running trace."""

import argparse
import json
import math
import time
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt


TRACE_PATH = Path("/tmp/velocity-planner.jsonl")
VIEWER_KEYS = {"left", "right", "up", "down", "home", "end", "f", "g", "G", "shift+g", "s", "r", "+", "=", "-", " ", "space"}


def reserve_viewer_keys():
    """Prevent Matplotlib's toolbar from also handling viewer shortcuts."""
    for setting in matplotlib.rcParams:
        if setting.startswith("keymap."):
            matplotlib.rcParams[setting] = [
                key for key in matplotlib.rcParams[setting] if key not in VIEWER_KEYS
            ]


def spline_segments(spline):
    """Evaluate each logged cubic only between its two adjacent path knots."""
    segments = []
    for coefficients in spline.get("segments", []):
        if len(coefficients) != 4 or any(
            len(coefficient) != 2 or any(value is None or not math.isfinite(value) for value in coefficient)
            for coefficient in coefficients
        ):
            continue
        xs, ys = [], []
        for step in range(21):
            u = step / 20
            x = coefficients[0][0] + u * (coefficients[1][0] + u * (coefficients[2][0] + u * coefficients[3][0]))
            y = coefficients[0][1] + u * (coefficients[1][1] + u * (coefficients[2][1] + u * coefficients[3][1]))
            if math.isfinite(x) and math.isfinite(y):
                xs.append(x)
                ys.append(y)
        if xs:
            segments.append((xs, ys))
    return segments


class TraceReader:
    def __init__(self, path: Path):
        self.path = path
        self.file = None
        self.file_id = None
        self.records = []
        self.generation = 0

    def read_available(self):
        changed = False
        if self.file is not None:
            try:
                info = self.path.stat()
            except FileNotFoundError:
                info = None
            if info is None or (info.st_dev, info.st_ino) != self.file_id or info.st_size < self.file.tell():
                self.file.close()
                self.file = None
                self.file_id = None
                self.records.clear()
                self.generation += 1
                changed = True
        if self.file is None:
            if not self.path.exists():
                return changed
            self.file = self.path.open(encoding="utf-8")
            info = self.path.stat()
            self.file_id = (info.st_dev, info.st_ino)
        while True:
            line_start = self.file.tell()
            line = self.file.readline()
            if not line:
                break
            if not line.endswith("\n"):
                self.file.seek(line_start)  # writer has not finished this record
                break
            try:
                self.records.append(json.loads(line))
                changed = True
            except json.JSONDecodeError as exc:
                print(f"Skipping invalid trace line at byte {line_start}: {exc}")
        return changed


class TraceViewer:
    def __init__(self, reader: TraceReader, live: bool):
        reserve_viewer_keys()
        self.reader = reader
        self.reader_generation = reader.generation
        self.following = live
        self.paused = False
        self.play_anchor_trace_ns = None
        self.play_anchor_wall_ns = None
        self.show_spline = True
        self.zoom_scale = 1.0
        self.pan_offset = (0.0, 0.0)
        self.pan_start = None
        self.record_index = 0
        self.change_index = 0
        self.figure, (self.map_ax, self.info_ax) = plt.subplots(
            1, 2, figsize=(14, 8), gridspec_kw={"width_ratios": [3, 1]}
        )
        self.figure.canvas.mpl_connect("key_press_event", self.on_key)
        self.figure.canvas.mpl_connect("scroll_event", self.on_scroll)
        self.figure.canvas.mpl_connect("button_press_event", self.on_press)
        self.figure.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.figure.canvas.mpl_connect("button_release_event", self.on_release)
        self.timer = self.figure.canvas.new_timer(interval=50)
        self.timer.add_callback(self.poll)
        self.timer.start()
        self.poll()
        self.draw()

    def poll(self):
        changed = self.reader.read_available()
        if self.reader_generation != self.reader.generation:
            self.reader_generation = self.reader.generation
            self.record_index = 0
            self.change_index = 0
            self.play_anchor_trace_ns = None
            self.play_anchor_wall_ns = None
        if not self.reader.records:
            self.record_index = 0
            self.change_index = 0
            self.play_anchor_trace_ns = None
            self.play_anchor_wall_ns = None
            if changed:
                self.draw()
            return
        if self.following:
            if self.play_anchor_trace_ns is None or self.record_index >= len(self.reader.records):
                # --live starts at the newest record; F from a selected record
                # instead sets an anchor there and plays toward the live edge.
                self.record_index = len(self.reader.records) - 1
                self.change_index = len(self.record["changes"])
                self.play_anchor_trace_ns = self.record["timestamp_ns"]
                self.play_anchor_wall_ns = time.monotonic_ns()
                changed = True
            else:
                target_trace_ns = self.play_anchor_trace_ns + time.monotonic_ns() - self.play_anchor_wall_ns
                while (self.record_index + 1 < len(self.reader.records)
                       and self.reader.records[self.record_index + 1]["timestamp_ns"] <= target_trace_ns):
                    self.record_index += 1
                    self.change_index = len(self.record["changes"])
                    changed = True
        else:
            self.record_index = min(self.record_index, len(self.reader.records) - 1)
        if changed:
            self.draw()

    @property
    def record(self):
        return self.reader.records[self.record_index]

    def on_key(self, event):
        if not self.reader.records and event.key not in ("f", " ", "space"):
            return
        if event.key in ("left", "right", "up", "down", "home", "end", "g", "G", "shift+g"):
            self.paused = False
        if event.key == "left":
            self.following = False
            self.record_index = max(0, self.record_index - 1)
            self.change_index = len(self.record["changes"])
        elif event.key == "right":
            self.following = False
            self.record_index = min(len(self.reader.records) - 1, self.record_index + 1)
            self.change_index = len(self.record["changes"])
        elif event.key == "down":
            self.following = False
            self.change_index = max(0, self.change_index - 1)
        elif event.key == "up":
            self.following = False
            self.change_index = min(len(self.record["changes"]), self.change_index + 1)
        elif event.key == "home":
            self.following = False
            self.change_index = 0
        elif event.key == "end":
            self.following = False
            self.change_index = len(self.record["changes"])
        elif event.key == "g":
            self.following = False
            self.record_index = 0
            self.change_index = len(self.record["changes"])
        elif event.key in ("G", "shift+g"):
            self.following = False
            self.record_index = len(self.reader.records) - 1
            self.change_index = len(self.record["changes"])
        elif event.key in ("f", " ", "space"):
            if event.key != "f" and self.following:
                self.following = False
                self.paused = True
                self.draw()
                return
            self.following = True
            self.paused = False
            self.reader.read_available()
            self.reader_generation = self.reader.generation
            if self.reader.records:
                self.record_index = min(self.record_index, len(self.reader.records) - 1)
                self.play_anchor_trace_ns = self.record["timestamp_ns"]
                self.play_anchor_wall_ns = time.monotonic_ns()
            else:
                self.record_index = 0
                self.change_index = 0
                self.play_anchor_trace_ns = None
                self.play_anchor_wall_ns = None
        elif event.key == "s":
            self.show_spline = not self.show_spline
        elif event.key in ("+", "="):
            self.zoom_scale = max(0.05, self.zoom_scale / 1.4)
        elif event.key == "-":
            self.zoom_scale = min(100.0, self.zoom_scale * 1.4)
        elif event.key == "r":
            self.zoom_scale = 1.0
            self.pan_offset = (0.0, 0.0)
        else:
            return
        self.draw()

    def on_scroll(self, event):
        if event.inaxes != self.map_ax or not self.reader.records:
            return
        if event.button == "up":
            self.zoom_scale = max(0.05, self.zoom_scale / 1.4)
        elif event.button == "down":
            self.zoom_scale = min(100.0, self.zoom_scale * 1.4)
        else:
            return
        self.draw()

    def on_press(self, event):
        if (event.button != 1 or event.inaxes != self.map_ax or not self.reader.records
                or self.map_ax.get_navigate_mode() is not None):
            return
        xlim = self.map_ax.get_xlim()
        ylim = self.map_ax.get_ylim()
        self.pan_start = (event.x, event.y, self.pan_offset,
                          (xlim[1] - xlim[0]) / self.map_ax.bbox.width,
                          (ylim[1] - ylim[0]) / self.map_ax.bbox.height)

    def on_motion(self, event):
        if self.pan_start is None:
            return
        start_x, start_y, (offset_x, offset_y), x_per_pixel, y_per_pixel = self.pan_start
        self.pan_offset = (offset_x - (event.x - start_x) * x_per_pixel,
                           offset_y - (event.y - start_y) * y_per_pixel)
        self.draw()

    def on_release(self, event):
        if event.button == 1:
            self.pan_start = None

    def draw(self):
        self.map_ax.clear()
        self.info_ax.clear()
        self.info_ax.axis("off")
        if not self.reader.records:
            self.map_ax.text(0.5, 0.5, "Waiting for trace records...", ha="center", va="center")
            self.figure.canvas.draw_idle()
            return

        record = self.record
        path = record["path_map"]
        if path:
            self.map_ax.scatter([p[0] for p in path], [p[1] for p in path],
                                color="#4d5a66", s=14, zorder=5)
        spline = record.get("spline", {})
        if self.show_spline:
            for spline_x, spline_y in spline_segments(spline):
                self.map_ax.plot(spline_x, spline_y, color="#ee821c", linewidth=1.0,
                                 alpha=0.65, zorder=2)

        start_index = record["start_path_index"]
        if start_index is not None and start_index < len(path):
            p = path[start_index]
            self.map_ax.scatter(p[0], p[1], color="#d425ad", marker="*", s=220,
                                edgecolors="black", label=f"first path point [{start_index}]", zorder=8)

        speeds = {}
        for change in record["changes"][:self.change_index]:
            speeds[change["path_index"]] = change["new_velocity"]
        for index, speed in speeds.items():
            if index < len(path) and speed is not None and math.isfinite(speed):
                p = path[index]
                self.map_ax.annotate(f"{speed:.1f}", p, xytext=(3, 4),
                                     textcoords="offset points", fontsize=7, color="#202020")
        if self.change_index:
            selected_change = record["changes"][self.change_index - 1]
            selected = selected_change["path_index"]
            if selected < len(path):
                p = path[selected]
                self.map_ax.scatter(p[0], p[1], facecolors="none", edgecolors="#00a9c5",
                                    linewidths=2.5, s=260, label=f"selected path point [{selected}]", zorder=7)
            sample_index = selected_change["index"]
            samples = spline.get("samples", [])
            if self.show_spline and sample_index < len(samples):
                p = samples[sample_index]["position"]
                if all(value is not None and math.isfinite(value) for value in p):
                    self.map_ax.scatter(p[0], p[1], color="#ee821c", marker="x", s=100,
                                        label="selected spline sample", zorder=9)

        car = record["car"]
        x, y = car["position_map"]
        hx, hy = car["heading_map"]
        self.map_ax.scatter(x, y, color="#d43c39", marker="o", s=100, label="car", zorder=6)
        if math.isfinite(hx) and math.isfinite(hy):
            extent = max([3.0] + [abs(p[0] - x) for p in path] + [abs(p[1] - y) for p in path])
            arrow_length = min(1.0, 0.12 * extent)
            self.map_ax.arrow(x, y, hx * arrow_length, hy * arrow_length,
                              width=0.012 * arrow_length, head_width=0.12 * arrow_length,
                              color="#d43c39", zorder=6)

        # Spline excursions are useful to see, but must not set the plot scale.
        view_points = [*path, [x, y]]
        x_values = [point[0] for point in view_points if math.isfinite(point[0])]
        y_values = [point[1] for point in view_points if math.isfinite(point[1])]
        if x_values and y_values:
            x_pad = max(0.5, (max(x_values) - min(x_values)) * 0.05)
            y_pad = max(0.5, (max(y_values) - min(y_values)) * 0.05)
            x_center = (min(x_values) + max(x_values)) / 2
            y_center = (min(y_values) + max(y_values)) / 2
            x_half = ((max(x_values) - min(x_values)) / 2 + x_pad) * self.zoom_scale
            y_half = ((max(y_values) - min(y_values)) / 2 + y_pad) * self.zoom_scale
            self.map_ax.set_xlim(x_center + self.pan_offset[0] - x_half,
                                 x_center + self.pan_offset[0] + x_half)
            self.map_ax.set_ylim(y_center + self.pan_offset[1] - y_half,
                                 y_center + self.pan_offset[1] + y_half)
        self.map_ax.set_aspect("equal", adjustable="box")
        self.map_ax.grid(alpha=0.25)
        self.map_ax.set_xlabel("map x (m)")
        self.map_ax.set_ylabel("map y (m)")
        self.map_ax.set_title(f"Iteration {record['iteration']}  |  {record['status']}")

        velocity = car["velocity_raw"]
        rows = [
            f"Record {self.record_index + 1}/{len(self.reader.records)}",
            f"Step {self.change_index}/{len(record['changes'])}",
            f"Mode: {'PAUSED' if self.paused else 'PLAYING 1x' if self.following and self.record_index < len(self.reader.records) - 1 else 'LIVE (waiting)' if self.following else 'MANUAL'}",
            f"Spline: {'shown' if self.show_spline else 'hidden'}",
            f"View scale: {self.zoom_scale:.2f}x",
            "Gray path points | Orange spline | Star first point",
            "",
            f"Car: ({x:.2f}, {y:.2f}) m",
            f"Heading: ({hx:.3f}, {hy:.3f})",
            f"Velocity raw: ({velocity[0]:.2f}, {velocity[1]:.2f}) m/s",
            f"Along heading: {format_value(record['longitudinal_velocity'])} m/s",
            f"Pursuit curvature: {record['pose_to_path_curvature']:.4f} 1/m",
            f"Start path index: {start_index}",
            "",
        ]
        if self.change_index:
            change = record["changes"][self.change_index - 1]
            rows.extend([
                f"Pass: {change['pass']}",
                f"path_points_[{change['index']}]",
                f"path_[{change['path_index']}]",
                f"Changed: {change.get('changed', True)}",
                f"Velocity: {format_value(change['old_velocity'])} → {format_value(change['new_velocity'])} m/s",
            ])
            samples = spline.get("samples", [])
            if change["index"] < len(samples):
                sample = samples[change["index"]]
                rows.append(f"Spline t: {format_value(sample.get('t'), 3)} m")
                rows.append(f"Spline curvature: {format_value(sample.get('curvature'), 5)} 1/m")
            if change["pass"] != "initial":
                rows.extend([
                    f"Source curvature: {format_value(change.get('curvature'), 5)} 1/m",
                    f"Lateral accel: {format_value(change.get('lateral_accel'))} m/s²",
                    f"Long accel available: {format_value(change.get('longitudinal_accel_available'))} m/s²",
                ])
            rows.append("")
        if self.change_index == len(record["changes"]):
            failure = record.get("failure")
            if failure:
                rows.extend(["Failure:", *[f"{k}: {v}" for k, v in failure.items()], ""])
            accel_calculation = record.get("accel_calculation")
            if accel_calculation:
                rows.extend([
                    f"Target speed: {format_value(accel_calculation['target_velocity'])} m/s",
                    f"Accel distance: {format_value(accel_calculation['distance'])} m",
                ])
            rows.extend([f"Final accel: {format_value(record['accel'])} m/s²",
                         f"Torque/wheel: {format_value(record['torque_per_wheel_nm'])} Nm", ""])
        rows.extend(["←/→  previous/next iteration", "↓/↑  previous/next step",
                     "G/Shift+G  first/last iteration", "Home/End  first/final step",
                     "Space  pause/resume; F  play from here", "S  show/hide spline",
                     "Scroll or +/-  zoom; left drag  pan; R  reset view"])
        self.info_ax.text(0.02, 0.98, "\n".join(rows), va="top", family="monospace", fontsize=9,
                          transform=self.info_ax.transAxes)
        self.figure.tight_layout()
        self.figure.canvas.draw_idle()


def format_value(value, digits=2):
    return "—" if value is None else f"{value:.{digits}f}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--live", action="store_true", help="Follow new records as they are written")
    args = parser.parse_args()
    if not args.live and not TRACE_PATH.exists():
        parser.error(f"trace does not exist: {TRACE_PATH}")
    viewer = TraceViewer(TraceReader(TRACE_PATH), args.live)
    plt.show()
    viewer.timer.stop()


if __name__ == "__main__":
    main()
