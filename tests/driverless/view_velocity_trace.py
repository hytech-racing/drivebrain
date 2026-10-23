"""Interactively inspect VelocityPlannerDebug messages recorded in a drivebrain MCAP."""

import argparse
import math
import struct
import time
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
from mcap.exceptions import McapError
from mcap.reader import make_reader
from mcap.records import Channel, Message, Schema
from mcap.stream_reader import StreamReader
from mcap_protobuf.decoder import DecoderFactory


TOPIC = "planning/velocity_debug"
RECORDINGS_DIR = Path(__file__).resolve().parents[2] / "recordings"
VIEWER_KEYS = {"left", "right", "up", "down", "home", "end", "f", "g", "G", "shift+g", "s", "r", "+", "=", "-", " ", "space", "b", "[", "]"}


def reserve_viewer_keys():
    """Prevent Matplotlib's toolbar from also handling viewer shortcuts."""
    for setting in matplotlib.rcParams:
        if setting.startswith("keymap."):
            matplotlib.rcParams[setting] = [
                key for key in matplotlib.rcParams[setting] if key not in VIEWER_KEYS
            ]


def load_records(path: Path):
    try:
        with path.open("rb") as f:
            reader = make_reader(f, decoder_factories=[DecoderFactory()])
            return [message for _, _, _, message in reader.iter_decoded_messages(topics=[TOPIC])]
    except McapError:
        pass
    # No valid footer: drivebrain is still writing this file or was killed. Walk it record by record
    # and keep everything up to the truncated tail.
    schemas, channels, records = {}, {}, []
    decoder_factory = DecoderFactory()
    with path.open("rb") as f:
        try:
            for record in StreamReader(f).records:
                if isinstance(record, Schema):
                    schemas[record.id] = record
                elif isinstance(record, Channel):
                    channels[record.id] = record
                elif isinstance(record, Message):
                    channel = channels.get(record.channel_id)
                    if channel is not None and channel.topic == TOPIC:
                        decode = decoder_factory.decoder_for(channel.message_encoding, schemas[channel.schema_id])
                        records.append(decode(record.data))
        except (McapError, EOFError, ValueError, struct.error):
            pass
    print(f"{path} is unfinished, showing the {len(records)} complete records")
    return sorted(records, key=lambda record: record.timestamp_ns)


def enum_name(message, field, value):
    return message.DESCRIPTOR.fields_by_name[field].enum_type.values_by_number[value].name


def optional(message, field):
    return getattr(message, field) if message.HasField(field) else None


def spline_segments(spline):
    """Evaluate each logged cubic only between its two adjacent path points."""
    segments = []
    for segment in spline.segments:
        c = segment.coefficients
        if len(c) != 4 or not all(math.isfinite(v.x) and math.isfinite(v.y) for v in c):
            continue
        xs, ys = [], []
        for step in range(21):
            u = step / 20
            xs.append(c[0].x + u * (c[1].x + u * (c[2].x + u * c[3].x)))
            ys.append(c[0].y + u * (c[1].y + u * (c[2].y + u * c[3].y)))
        segments.append((xs, ys))
    return segments


def velocities_after(record, step_count):
    """path_points velocities after applying the first step_count changes."""
    speeds = [point.velocity_mps for point in record.path_points]
    for change in record.changes[:step_count]:
        if change.index < len(speeds):
            speeds[change.index] = change.new_velocity_mps
    return speeds


class TraceViewer:
    def __init__(self, records):
        reserve_viewer_keys()
        self.records = records
        self.playing = False
        self.direction = 1
        self.speed = 1.0
        self.play_anchor_trace_ns = None
        self.play_anchor_wall_ns = None
        self.show_spline = True
        self.zoom_scale = 1.0
        self.pan_offset = (0.0, 0.0)
        self.pan_start = None
        self.record_index = 0
        self.change_index = len(self.record.changes)
        self.figure, (self.map_ax, self.info_ax) = plt.subplots(
            1, 2, figsize=(14, 8), gridspec_kw={"width_ratios": [3, 1]}
        )
        self.figure.canvas.mpl_connect("key_press_event", self.on_key)
        self.figure.canvas.mpl_connect("scroll_event", self.on_scroll)
        self.figure.canvas.mpl_connect("button_press_event", self.on_press)
        self.figure.canvas.mpl_connect("motion_notify_event", self.on_motion)
        self.figure.canvas.mpl_connect("button_release_event", self.on_release)
        self.timer = self.figure.canvas.new_timer(interval=50)
        self.timer.add_callback(self.tick)
        self.timer.start()
        self.draw()

    @property
    def record(self):
        return self.records[self.record_index]

    def go_to(self, index):
        self.record_index = max(0, min(len(self.records) - 1, index))
        self.change_index = len(self.record.changes)

    def anchor_playback(self):
        self.play_anchor_trace_ns = self.record.timestamp_ns
        self.play_anchor_wall_ns = time.monotonic_ns()

    def tick(self):
        if not self.playing:
            return
        elapsed_ns = (time.monotonic_ns() - self.play_anchor_wall_ns) * self.speed
        target_trace_ns = self.play_anchor_trace_ns + self.direction * elapsed_ns
        index = self.record_index
        if self.direction > 0:
            while index + 1 < len(self.records) and self.records[index + 1].timestamp_ns <= target_trace_ns:
                index += 1
        else:
            while index > 0 and self.records[index - 1].timestamp_ns >= target_trace_ns:
                index -= 1
        at_end = index == (len(self.records) - 1 if self.direction > 0 else 0)
        if index != self.record_index or at_end:
            self.go_to(index)
            self.playing = not at_end
            self.draw()

    def on_key(self, event):
        if event.key in ("left", "right", "up", "down", "home", "end", "g", "G", "shift+g"):
            self.playing = False
        if event.key == "left":
            self.go_to(self.record_index - 1)
        elif event.key == "right":
            self.go_to(self.record_index + 1)
        elif event.key == "down":
            self.change_index = max(0, self.change_index - 1)
        elif event.key == "up":
            self.change_index = min(len(self.record.changes), self.change_index + 1)
        elif event.key == "home":
            self.change_index = 0
        elif event.key == "end":
            self.change_index = len(self.record.changes)
        elif event.key == "g":
            self.go_to(0)
        elif event.key in ("G", "shift+g"):
            self.go_to(len(self.records) - 1)
        elif event.key in ("[", "]"):
            self.speed = min(32.0, self.speed * 2) if event.key == "]" else max(0.125, self.speed / 2)
            if self.playing:
                self.anchor_playback()
        elif event.key in (" ", "space"):
            self.playing = not self.playing
            if self.playing:
                self.anchor_playback()
        elif event.key in ("f", "b"):
            self.direction = 1 if event.key == "f" else -1
            self.playing = True
            self.anchor_playback()
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
        if event.inaxes != self.map_ax:
            return
        if event.button == "up":
            self.zoom_scale = max(0.05, self.zoom_scale / 1.4)
        elif event.button == "down":
            self.zoom_scale = min(100.0, self.zoom_scale * 1.4)
        else:
            return
        self.draw()

    def on_press(self, event):
        if event.button != 1 or event.inaxes != self.map_ax or self.map_ax.get_navigate_mode() is not None:
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

        record = self.record
        points = [(p.position_m.x, p.position_m.y) for p in record.path_points]
        if points:
            self.map_ax.scatter([p[0] for p in points], [p[1] for p in points],
                                color="#4d5a66", s=14, zorder=5)
            self.map_ax.scatter(*points[0], color="#d425ad", marker="*", s=220,
                                edgecolors="black", label="first path point", zorder=8)
        if self.show_spline:
            for spline_x, spline_y in spline_segments(record.spline):
                self.map_ax.plot(spline_x, spline_y, color="#ee821c", linewidth=1.0,
                                 alpha=0.65, zorder=2)

        speeds = velocities_after(record, self.change_index)
        for point, speed in zip(points, speeds):
            if math.isfinite(speed):
                self.map_ax.annotate(f"{speed:.1f}", point, xytext=(3, 4),
                                     textcoords="offset points", fontsize=7, color="#202020")
        change = record.changes[self.change_index - 1] if self.change_index else None
        if change is not None and change.index < len(points):
            self.map_ax.scatter(*points[change.index], facecolors="none", edgecolors="#00a9c5",
                                linewidths=2.5, s=260, label=f"path_points[{change.index}]", zorder=7)

        x, y = record.car_position_m.x, record.car_position_m.y
        hx, hy = record.car_heading.x, record.car_heading.y
        self.map_ax.scatter(x, y, color="#d43c39", marker="o", s=100, label="car", zorder=6)
        if math.isfinite(hx) and math.isfinite(hy):
            extent = max([3.0] + [abs(p[0] - x) for p in points] + [abs(p[1] - y) for p in points])
            arrow_length = min(1.0, 0.12 * extent)
            self.map_ax.arrow(x, y, hx * arrow_length, hy * arrow_length,
                              width=0.012 * arrow_length, head_width=0.12 * arrow_length,
                              color="#d43c39", zorder=6)

        # Spline excursions are useful to see, but must not set the plot scale.
        view_points = [*points, (x, y)]
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
        status = enum_name(record, "status", record.status)
        self.map_ax.set_title(f"Iteration {record.iteration}  |  {status}")

        if self.playing:
            mode = f"{'PLAYING' if self.direction > 0 else 'REVERSE'} {self.speed:g}x"
        else:
            mode = "PAUSED"
        velocity = record.car_velocity_raw_mps
        rows = [
            f"Record {self.record_index + 1}/{len(self.records)}",
            f"Step {self.change_index}/{len(record.changes)}",
            f"Mode: {mode}",
            f"Spline: {'shown' if self.show_spline else 'hidden'}",
            f"View scale: {self.zoom_scale:.2f}x",
            "Gray path points | Orange spline | Star first point",
            "",
            f"Car: ({x:.2f}, {y:.2f}) m",
            f"Heading: ({hx:.3f}, {hy:.3f})",
            f"Velocity raw: ({velocity.x:.2f}, {velocity.y:.2f}) m/s",
            f"Along heading: {format_value(optional(record, 'longitudinal_velocity_mps'))} m/s",
            f"mu: {record.mu:.2f}  max v: {record.max_car_velocity_mps:.2f} m/s",
            "",
        ]
        if change is not None:
            point = record.path_points[change.index] if change.index < len(record.path_points) else None
            old_velocity = velocities_after(record, self.change_index - 1)[change.index] if point else None
            arc_length = sum(math.dist(points[i - 1], points[i]) for i in range(1, change.index + 1)) if point else None
            rows.extend([
                f"Pass: {enum_name(change, 'pass', getattr(change, 'pass'))}",
                f"path_points[{change.index}]  s = {format_value(arc_length)} m",
                f"Changed: {change.changed}",
                f"Velocity: {format_value(old_velocity)} → {format_value(change.new_velocity_mps)} m/s",
                f"Curvature limit: {format_value(point.velocity_mps if point else None)} m/s",
                f"Point curvature: {format_value(point.curvature if point else None, 5)} 1/m",
                f"Source curvature: {format_value(change.curvature, 5)} 1/m",
                f"Lateral accel: {format_value(change.lateral_accel_mps2)} m/s²"
                + ("  EXCEEDED" if change.lateral_accel_exceeded else ""),
                f"Long accel available: {format_value(change.longitudinal_accel_available_mps2)} m/s²",
                f"Allowed by accel: {format_value(change.velocity_allowed_by_accel_mps)} m/s",
                "",
            ])
        if self.change_index == len(record.changes):
            rows.extend([
                f"Target speed: {format_value(optional(record, 'target_velocity_mps'))} m/s",
                f"Final accel: {format_value(optional(record, 'accel_mps2'))} m/s²",
                f"Torque/wheel: {format_value(optional(record, 'torque_per_wheel_nm'))} Nm",
                "",
            ])
        rows.extend(["←/→  previous/next iteration", "↓/↑  previous/next step",
                     "G/Shift+G  first/last iteration", "Home/End  first/final step",
                     "Space  pause/resume; F/B  play forward/back",
                     "[/]  halve/double playback speed", "S  show/hide spline",
                     "Scroll or +/-  zoom; left drag  pan; R  reset view"])
        self.info_ax.text(0.02, 0.98, "\n".join(rows), va="top", family="monospace", fontsize=9,
                          transform=self.info_ax.transAxes)
        self.figure.tight_layout()
        self.figure.canvas.draw_idle()


def format_value(value, digits=2):
    return "—" if value is None else f"{value:.{digits}f}"


def latest_recording():
    recordings = sorted(RECORDINGS_DIR.glob("*.mcap"), key=lambda path: path.stat().st_mtime)
    return recordings[-1] if recordings else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mcap", nargs="?", type=Path,
                        help=f"recording to open (default: newest in {RECORDINGS_DIR})")
    args = parser.parse_args()
    path = args.mcap or latest_recording()
    if path is None or not path.exists():
        parser.error(f"no recording given and none found in {RECORDINGS_DIR}" if path is None
                     else f"recording does not exist: {path}")
    records = load_records(path)
    if not records:
        parser.error(f"no {TOPIC} messages in {path}")
    viewer = TraceViewer(records)
    plt.show()
    viewer.timer.stop()


if __name__ == "__main__":
    main()
