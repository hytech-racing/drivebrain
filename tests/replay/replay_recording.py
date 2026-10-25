#!/usr/bin/env python3
"""Replay a Drivebrain MCAP into a HOOTL build over the SimComms ZMQ sockets.

Vehicle messages (OMSData, VNData, camera frames, ...) go to the vehicle socket as
[type name, topic, bytes] and are timestamped by drivebrain on receive, exactly like
the live car. Point clouds go to the lidar socket with their timestamp rewritten to
send time so every message shares the live host clock, and with a per-point `t` field
(seconds before the scan end, recovered from each point's azimuth) when the recording
only logged xyz, so deskewing works. Drivebrain logs everything it receives on the
original topics, so a recording of a replay can itself be replayed.

Usage:
    .venv/bin/python -I replay_recording.py recording.mcap --start 10 --duration 60 [--topics a,b,glob*]
"""

from __future__ import annotations

import argparse
from fnmatch import fnmatch
import json
import sys
import time

from foxglove_schemas_protobuf.PackedElementField_pb2 import PackedElementField
from foxglove_schemas_protobuf.PointCloud_pb2 import PointCloud
from mcap.reader import make_reader
import numpy as np
import zmq

VEHICLE_PORT = 6767
LIDAR_PORT = 1155
LIDAR_TOPICS = {"ouster/points", "PointCloud"}
CALIBRATION_TOPIC = "ouster/calibration"
DEFAULT_TOPICS = ["PointCloud", "ouster/points", "OMSData", "blackfly_*/detections", "steering_data"]


class ScanTiming:
    """Column (= time within the scan) of each point from its azimuth, using the sensor metadata.

    The scan sweeps azimuth clockwise from +180 deg at column 0; each beam carries its own
    azimuth offset, so the beam row is recovered from elevation first.
    """

    def __init__(self, metadata_json: str):
        meta = json.loads(metadata_json)
        fmt = meta["lidar_data_format"]
        beams = meta["beam_intrinsics"]
        self.columns = int(fmt["columns_per_frame"])
        self.scan_period_s = 1.0 / float(fmt["fps"])
        self.column_deg = 360.0 / self.columns
        self.beam_azimuth = np.asarray(beams["beam_azimuth_angles"], float)
        self.beam_altitude = np.asarray(beams["beam_altitude_angles"], float)
        self.beam_origin_m = float(beams["lidar_origin_to_beam_origin_mm"]) / 1000.0
        lidar_to_sensor = np.asarray(meta["lidar_intrinsics"]["lidar_to_sensor_transform"], float).reshape(4, 4)
        self.z_offset_m = lidar_to_sensor[2, 3] / 1000.0
        self.column0_azimuth = (360.0 - self.beam_azimuth) % 360.0 - 180.0
        self._altitude_sorted = np.sort(self.beam_altitude)
        self._altitude_order = np.argsort(self.beam_altitude)

    def seconds_before_scan_end(self, xyz: np.ndarray) -> np.ndarray:
        x, y, z = (xyz[:, i].astype(np.float64) for i in range(3))
        horizontal = np.hypot(x, y)
        elevation = np.degrees(np.arctan2(z - self.z_offset_m, np.maximum(horizontal - self.beam_origin_m, 1e-3)))
        upper = np.clip(np.searchsorted(self._altitude_sorted, elevation), 1, len(self.beam_altitude) - 1)
        nearer_lower = np.abs(elevation - self._altitude_sorted[upper - 1]) < np.abs(elevation - self._altitude_sorted[upper])
        row = self._altitude_order[np.where(nearer_lower, upper - 1, upper)]
        azimuth = np.degrees(np.arctan2(y, x))
        column = ((self.column0_azimuth[row] - azimuth) % 360.0) / self.column_deg
        return ((column + 0.5) / self.columns - 1.0) * self.scan_period_s


def with_time_field(cloud: PointCloud, timing: ScanTiming | None) -> PointCloud:
    if any(field.name == "t" for field in cloud.fields):
        return cloud
    xyz = np.frombuffer(cloud.data, dtype="<f4").reshape(-1, 3)
    if timing is None:
        # azimuth only, no beam offsets: column error up to ~12 columns (1.2 ms)
        azimuth = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
        column = ((180.0 - azimuth) % 360.0) / (360.0 / 1024)
        t = ((column + 0.5) / 1024 - 1.0) * 0.1
    else:
        t = timing.seconds_before_scan_end(xyz)
    out = PointCloud()
    out.CopyFrom(cloud)
    out.point_stride = 16
    field = out.fields.add()
    field.name = "t"
    field.offset = 12
    field.type = PackedElementField.FLOAT32
    out.data = np.ascontiguousarray(np.column_stack([xyz, t.astype("<f4")]), dtype="<f4").tobytes()
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mcap")
    parser.add_argument("--start", type=float, default=0.0, help="seconds after the first message")
    parser.add_argument("--duration", type=float, default=None, help="seconds to replay (default: to the end)")
    parser.add_argument("--rate", type=float, default=1.0, help="playback speed multiplier")
    parser.add_argument("--topics", default=",".join(DEFAULT_TOPICS), help="comma-separated topics (globs) to replay")
    parser.add_argument("--no-time-field", action="store_true", help="send point clouds as recorded (xyz only)")
    parser.add_argument("--bind", default="tcp://*", help="ZMQ bind address prefix")
    args = parser.parse_args()
    patterns = [t for t in args.topics.split(",") if t]

    ctx = zmq.Context()
    vehicle = ctx.socket(zmq.PUSH)
    vehicle.set(zmq.SNDHWM, 10000)
    vehicle.bind(f"{args.bind}:{VEHICLE_PORT}")
    lidar = ctx.socket(zmq.PUSH)
    lidar.set(zmq.SNDHWM, 2)
    lidar.bind(f"{args.bind}:{LIDAR_PORT}")
    print(f"replaying {patterns} on {args.bind}:{VEHICLE_PORT} (vehicle) and :{LIDAR_PORT} (lidar); waiting for drivebrain")

    with open(args.mcap, "rb") as f:
        reader = make_reader(f)
        summary = reader.get_summary()
        topics = sorted({c.topic for c in summary.channels.values() if any(fnmatch(c.topic, p) for p in patterns)})
        print("topics:", topics)
        first_ns = summary.statistics.message_start_time
        start_ns = first_ns + int(args.start * 1e9)
        end_ns = start_ns + int(args.duration * 1e9) if args.duration else summary.statistics.message_end_time

        # the calibration is a single message at the start of the recording: use it for the
        # per-point timing and forward it so the new recording carries it too
        timing = None
        for schema, channel, message in reader.iter_messages(topics=[CALIBRATION_TOPIC]):
            if not args.no_time_field:
                from foxglove_schemas_protobuf.RawImage_pb2 import RawImage
                calibration = RawImage()
                calibration.ParseFromString(message.data)
                timing = ScanTiming(calibration.data.decode("utf-8"))
            break
        if timing is None and not args.no_time_field:
            print("no ouster/calibration message: per-point times use azimuth only", file=sys.stderr)
        topics = [t for t in topics if t != CALIBRATION_TOPIC]

        wall_start = time.monotonic()
        counts: dict[str, int] = {}
        for schema, channel, message in reader.iter_messages(topics=topics, start_time=start_ns, end_time=end_ns):
            due = wall_start + (message.log_time - start_ns) / 1e9 / args.rate
            delay = due - time.monotonic()
            if delay > 0:
                time.sleep(delay)

            if channel.topic in LIDAR_TOPICS:
                cloud = PointCloud()
                cloud.ParseFromString(message.data)
                if not args.no_time_field:
                    cloud = with_time_field(cloud, timing)
                now_ns = time.time_ns()
                cloud.timestamp.seconds = now_ns // 1_000_000_000
                cloud.timestamp.nanos = now_ns % 1_000_000_000
                try:
                    lidar.send(cloud.SerializeToString(), zmq.NOBLOCK)
                except zmq.Again:
                    counts["dropped_scans"] = counts.get("dropped_scans", 0) + 1
                    continue
            else:
                vehicle.send_multipart([schema.name.encode(), channel.topic.encode(), message.data])
            counts[channel.topic] = counts.get(channel.topic, 0) + 1

    print("sent", counts)
    vehicle.close(linger=1000)
    lidar.close(linger=1000)
    return 0


if __name__ == "__main__":
    sys.exit(main())
 