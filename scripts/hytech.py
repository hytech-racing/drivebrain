#!/usr/bin/env python3
"""HyTech MCAP tool.

    hytech recover recording.mcap     recover if damaged, then decompress and trim
    hytech decompress recording.mcap  decompress the whole recording
    hytech trim recording.mcap        trim without decompressing
    hytech strip recording.mcap       keep only data for driven analysis

Decompressing turns camera <name>/raw messages into ordinary foxglove.RawImage
Bayer RGGB8 messages and Ouster range images into foxglove.PointCloud messages
on ouster/points. When a recording has no Ouster calibration message, the
metadata captured in 235.mcap is used. All other messages, plus attachments
and metadata, are copied. Output: recording_decompressed.mcap.

Trimming keeps the time the car spent in READY_TO_DRIVE
(hytech.car_states.vehicle_state == 3), plus 5 seconds before and after. Each
separate drive is written to its own file: recording_0.mcap,
recording_1.mcap, ... Drives whose padded windows overlap are merged into one
file. Single-message channels (such as the drivebrain configuration) and
Ouster calibration are copied into every file. If the recording has no
car_states messages, recover keeps the whole recording.

Stripping keeps only what driven analysis needs. It removes cameras
(blackfly_*), lidar (ouster/*, PointCloud), Foxglove visualization
(SceneUpdate, tf), and driverless-only data (dv_msgs.*, slam, KrakenInfo,
drivebrain_steering_input), and copies everything else unchanged.
Output: recording_stripped.mcap.

Install Python dependencies with:
    python3 -m pip install -r requirements.txt
and install the MCAP CLI separately.

The input is never modified. If MCAP recovery is needed, the recovered copy
is kept beside the output. Recovery may discard unreadable records.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
from pathlib import Path
import shutil
import subprocess

from foxglove_schemas_protobuf.PackedElementField_pb2 import PackedElementField
from foxglove_schemas_protobuf.PointCloud_pb2 import PointCloud
from foxglove_schemas_protobuf.RawImage_pb2 import RawImage
from google.protobuf import descriptor_pb2
from mcap.reader import make_reader
from mcap.writer import Writer
from mcap_protobuf.decoder import DecoderFactory
import numpy as np
from ouster.sdk import core
import zstandard as zstd


CAMERA_ENCODING = "bayer_rggb8"
CALIBRATION_TOPIC = "ouster/calibration"
FALLBACK_CALIBRATION = Path(__file__).with_name("ouster_calibration.json")
RANGE_TOPIC = "ouster/range"
POINTS_TOPIC = "ouster/points"
MAX_FRAME_BYTES = 512 * 1024 * 1024

# strip removes camera, lidar, visualization, and driverless-only data
STRIP_TOPIC_PREFIXES = ("blackfly_", "ouster/", "slam/", "slam_")
STRIP_TOPICS = {"filtered_cones", "tf", "KrakenInfo", "drivebrain_steering_input"}
# foxglove.* covers PointCloud, SceneUpdate, FrameTransform, and images
STRIP_SCHEMA_PREFIXES = ("foxglove.", "dv_msgs.")


def is_stripped(schema, topic: str) -> bool:
    return (
        topic.startswith(STRIP_TOPIC_PREFIXES)
        or topic in STRIP_TOPICS
        or (schema is not None and schema.name.startswith(STRIP_SCHEMA_PREFIXES))
    )

CAR_STATES_SCHEMA = "hytech.car_states"
READY_TO_DRIVE = 3
TRIM_PADDING_NS = 5_000_000_000

Window = tuple[int, int]  # inclusive [start, end] log times in nanoseconds


def decompressed_bytes(data: bytes, expected_size: int) -> bytes:
    if not 0 < expected_size <= MAX_FRAME_BYTES:
        raise ValueError(f"invalid decompressed size: {expected_size}")
    result = zstd.ZstdDecompressor().decompress(data, max_output_size=expected_size)
    if len(result) != expected_size:
        raise ValueError(f"expected {expected_size} bytes, got {len(result)}")
    return result


def decode_bayer(data: bytes, width: int, height: int) -> bytes:
    """Undo Zstd, cumulative uint8 differences, and RGGB plane splitting."""
    count = width * height
    differences = np.frombuffer(decompressed_bytes(data, count), dtype=np.uint8)
    planes = np.cumsum(differences, dtype=np.uint8)
    image = np.empty((height, width), dtype=np.uint8)
    offset = 0
    for row_start, col_start in ((0, 0), (0, 1), (1, 0), (1, 1)):
        view = image[row_start::2, col_start::2]
        size = view.size
        view[:] = planes[offset : offset + size].reshape(view.shape)
        offset += size
    return image.tobytes()


def decode_ranges(data: bytes, width: int, height: int) -> np.ndarray:
    """Undo Zstd and cumulative uint32 differences (modulo 2**32)."""
    differences = np.frombuffer(
        decompressed_bytes(data, width * height * 4), dtype="<u4"
    )
    return np.cumsum(differences, dtype=np.uint32).reshape(height, width)


def decode_proto(factory: DecoderFactory, schema, channel, data: bytes):
    if schema is None or schema.encoding != "protobuf" or channel.message_encoding != "protobuf":
        raise ValueError(f"{channel.topic} does not have a protobuf schema")
    decoder = factory.decoder_for(channel.message_encoding, schema)
    return decoder(data)


def calibration_lut(metadata_json: str):
    """Use the sensor's own metadata to map range pixels into XYZ points."""
    info = core.SensorInfo(metadata_json)
    # This matches C++ XYZLut(info, true) and cartesianT<float>.
    return core.XYZLutFloat(info, use_extrinsics=True)


def pointcloud_schema(message_class=PointCloud) -> bytes:
    """Build an MCAP protobuf descriptor set for a Foxglove message class."""
    files = {}

    def add_file(file):
        if file.name in files:
            return
        for dependency in file.dependencies:
            add_file(dependency)
        proto = descriptor_pb2.FileDescriptorProto()
        file.CopyToProto(proto)
        files[file.name] = proto

    add_file(message_class.DESCRIPTOR.file)
    return descriptor_pb2.FileDescriptorSet(file=list(files.values())).SerializeToString()


def ready_to_drive_windows(path: Path, padding_ns: int = TRIM_PADDING_NS) -> list[Window] | None:
    """Find padded, merged log-time windows around READY_TO_DRIVE.

    Returns None when the recording has no car_states messages, and an empty
    list when it has them but the car never reached READY_TO_DRIVE.
    """
    with path.open("rb") as source:
        reader = make_reader(source)
        factory = DecoderFactory()
        topics = None
        summary = reader.get_summary()
        if summary is not None:
            topics = [
                channel.topic
                for channel in summary.channels.values()
                if channel.schema_id in summary.schemas
                and summary.schemas[channel.schema_id].name == CAR_STATES_SCHEMA
            ]
            if not topics:
                return None

        spans: list[Window] = []
        ready_since = None
        last_time = None
        for schema, channel, message in reader.iter_messages(topics=topics, log_time_order=True):
            if schema is None or schema.name != CAR_STATES_SCHEMA:
                continue
            state = decode_proto(factory, schema, channel, message.data).vehicle_state
            if state == READY_TO_DRIVE:
                if ready_since is None:
                    ready_since = message.log_time
            elif ready_since is not None:
                spans.append((ready_since, message.log_time))
                ready_since = None
            last_time = message.log_time

    if last_time is None:
        return None
    if ready_since is not None:  # still ready when the recording ended
        spans.append((ready_since, last_time))

    windows: list[Window] = []
    for start, end in spans:
        start, end = max(0, start - padding_ns), end + padding_ns
        if windows and start <= windows[-1][1]:
            windows[-1] = (windows[-1][0], max(windows[-1][1], end))
        else:
            windows.append((start, end))
    return windows


class Output:
    """One output MCAP, optionally limited to a log-time window."""

    def __init__(self, path: Path, window: Window | None, profile: str):
        self.path = path
        self.window = window
        self.file = path.open("xb")
        self.writer = Writer(self.file)
        self.writer.start(profile=profile, library="drivebrain-mcap-decompress")
        self.schema_ids: dict[int, int] = {}
        self.channel_ids: dict[int, int] = {}
        self.derived_channel_ids: dict[object, int] = {}

    def clamp(self, log_time: int) -> int:
        """Move a copied-everywhere message into this file's time range."""
        if self.window is None:
            return log_time
        return min(max(log_time, self.window[0]), self.window[1])

    def channel(self, schema, channel) -> int:
        if channel.id not in self.channel_ids:
            schema_id = 0
            if schema is not None:
                if schema.id not in self.schema_ids:
                    self.schema_ids[schema.id] = self.writer.register_schema(
                        name=schema.name, encoding=schema.encoding, data=schema.data
                    )
                schema_id = self.schema_ids[schema.id]
            self.channel_ids[channel.id] = self.writer.register_channel(
                topic=channel.topic,
                message_encoding=channel.message_encoding,
                schema_id=schema_id,
                metadata=channel.metadata,
            )
        return self.channel_ids[channel.id]

    def derived_channel(self, key, topic: str, message_class) -> int:
        """Register a channel for messages this script creates (point clouds, video)."""
        if key not in self.derived_channel_ids:
            schema_id = self.writer.register_schema(
                name=message_class.DESCRIPTOR.full_name,
                encoding="protobuf",
                data=pointcloud_schema(message_class),
            )
            self.derived_channel_ids[key] = self.writer.register_channel(
                topic=topic, message_encoding="protobuf", schema_id=schema_id
            )
        return self.derived_channel_ids[key]


def convert(
    input_path: Path,
    outputs: list[tuple[Path, Window | None]],
    *,
    decompress=True,
    decode_video=False,
    strip=False,
) -> dict[str, int]:
    """Copy input_path into each output, keeping messages inside its window.

    Windows must be sorted and non-overlapping. A window of None keeps everything.
    With decompress=False, payloads are copied unchanged. With strip=True,
    camera, lidar, visualization, and driverless topics are dropped.
    """
    counts = {"cameras": 0, "ranges": 0, "copied": 0, "stripped": 0}
    opened: list[Output] = []
    try:
        with input_path.open("rb") as source:
            reader = make_reader(source)
            decoder_factory = DecoderFactory()

            lut = None
            embedded_calibration = next(reader.iter_messages(topics=[CALIBRATION_TOPIC]), None)
            if decompress and embedded_calibration is None:
                lut = calibration_lut(FALLBACK_CALIBRATION.read_text(encoding="utf-8"))
                print(f"No {CALIBRATION_TOPIC} message; using {FALLBACK_CALIBRATION}")
            elif decompress:
                calibration_schema, calibration_channel, calibration_message = embedded_calibration
                calibration = decode_proto(
                    decoder_factory, calibration_schema, calibration_channel, calibration_message.data
                )
                if calibration.encoding != "ouster_sensor_metadata_json":
                    raise ValueError("unexpected Ouster calibration encoding")
                lut = calibration_lut(calibration.data.decode("utf-8"))

            # Channels with a single message (e.g. configuration) are usually
            # logged once at startup, so copy them into every output.
            single_message_channels: set[int] = set()
            summary = reader.get_summary()
            if summary is not None and summary.statistics is not None:
                single_message_channels = {
                    channel_id
                    for channel_id, count in summary.statistics.channel_message_counts.items()
                    if count == 1
                }

            profile = reader.get_header().profile
            for path, window in outputs:
                opened.append(Output(path, window, profile))
            windowed = all(output.window is not None for output in opened)
            window_starts = [output.window[0] for output in opened] if windowed else []

            def targets_for(log_time: int) -> list[Output]:
                if not windowed:
                    return opened
                index = bisect_right(window_starts, log_time) - 1
                if index >= 0 and log_time <= opened[index].window[1]:
                    return [opened[index]]
                return []

            for schema, channel, message in reader.iter_messages(log_time_order=False):
                topic = channel.topic
                if strip and is_stripped(schema, topic):
                    counts["stripped"] += 1
                    continue
                payload = message.data
                copy_everywhere = topic == CALIBRATION_TOPIC or channel.id in single_message_channels
                targets = opened if copy_everywhere else targets_for(message.log_time)
                if not targets:
                    continue  # outside every window; skip before decompressing

                # (derived channel key, topic, message class) for created messages,
                # or None to write the (possibly rewritten) payload on the original channel.
                derived = None
                try:
                    if not decompress:
                        counts["copied"] += 1

                    elif topic == CALIBRATION_TOPIC:
                        calibration = decode_proto(decoder_factory, schema, channel, payload)
                        if calibration.encoding != "ouster_sensor_metadata_json":
                            raise ValueError("unexpected Ouster calibration encoding")
                        lut = calibration_lut(calibration.data.decode("utf-8"))

                    elif decode_video and topic.startswith("blackfly_") and topic.endswith("/h264"):
                        import av

                        video = decode_proto(decoder_factory, schema, channel, payload)
                        if video.format != "h264":
                            raise ValueError(f"unexpected video format {video.format!r}")
                        codec = av.CodecContext.create("h264", "r")
                        frames = codec.decode(av.Packet(video.data)) + codec.decode(None)
                        if len(frames) != 1:
                            raise ValueError(f"expected one video frame, got {len(frames)}")
                        bgr = frames[0].to_ndarray(format="bgr24")
                        image = RawImage()
                        image.timestamp.seconds = video.timestamp.seconds
                        image.timestamp.nanos = video.timestamp.nanos
                        image.frame_id = video.frame_id
                        image.height, image.width = bgr.shape[:2]
                        image.encoding = "bgr8"
                        image.step = image.width * 3
                        image.data = bgr.tobytes()
                        payload = image.SerializeToString()
                        derived = (channel.id, topic.removesuffix("/h264") + "/raw", RawImage)
                        counts["cameras"] += 1

                    elif topic.startswith("blackfly_") and topic.endswith("/raw"):
                        image = decode_proto(decoder_factory, schema, channel, payload)
                        if image.encoding == CAMERA_ENCODING:
                            pass  # Already converted.
                        elif image.encoding:
                            raise ValueError(f"unexpected camera encoding {image.encoding!r}")
                        else:
                            image.data = decode_bayer(image.data, image.width, image.height)
                            image.encoding = CAMERA_ENCODING
                            image.step = image.width
                            payload = image.SerializeToString()
                            counts["cameras"] += 1

                    elif topic == RANGE_TOPIC:
                        image = decode_proto(decoder_factory, schema, channel, payload)
                        if image.encoding:
                            raise ValueError(f"unexpected range encoding {image.encoding!r}")
                        if (image.width, image.height) != (lut.w, lut.h):
                            raise ValueError(
                                f"range size {image.width}x{image.height} differs "
                                f"from calibration {lut.w}x{lut.h}"
                            )
                        ranges = decode_ranges(image.data, image.width, image.height)
                        xyz = lut(ranges)
                        # A zero range means no return. Keep every real return.
                        points = np.ascontiguousarray(xyz[ranges != 0], dtype="<f4")
                        cloud = PointCloud()
                        cloud.timestamp.seconds = image.timestamp.seconds
                        cloud.timestamp.nanos = image.timestamp.nanos
                        cloud.frame_id = image.frame_id or "os_sensor"
                        cloud.pose.orientation.w = 1.0
                        cloud.point_stride = 12
                        for offset, name in enumerate(("x", "y", "z")):
                            field = cloud.fields.add()
                            field.name = name
                            field.offset = offset * 4
                            field.type = PackedElementField.FLOAT32
                        cloud.data = points.tobytes()
                        payload = cloud.SerializeToString()
                        derived = ("points", POINTS_TOPIC, PointCloud)
                        counts["ranges"] += 1

                    elif not (topic.startswith("blackfly_") and topic.endswith("/raw")):
                        counts["copied"] += 1
                except Exception as exc:
                    raise ValueError(f"{topic} at {message.log_time}: {exc}") from exc

                for output in targets:
                    channel_id = (
                        output.channel(schema, channel)
                        if derived is None
                        else output.derived_channel(*derived)
                    )
                    log_time = output.clamp(message.log_time) if copy_everywhere else message.log_time
                    output.writer.add_message(
                        channel_id=channel_id,
                        log_time=log_time,
                        publish_time=message.publish_time,
                        sequence=message.sequence,
                        data=payload,
                    )

            attachments = list(reader.iter_attachments())
            metadata_records = list(reader.iter_metadata())
            for output in opened:
                for attachment in attachments:
                    output.writer.add_attachment(
                        create_time=attachment.create_time,
                        log_time=attachment.log_time,
                        name=attachment.name,
                        media_type=attachment.media_type,
                        data=attachment.data,
                    )
                for metadata in metadata_records:
                    output.writer.add_metadata(name=metadata.name, data=metadata.metadata)
                output.writer.finish()
                output.file.close()

        return counts
    except Exception:
        # only delete files this run created
        for output in opened:
            output.file.close()
            output.path.unlink(missing_ok=True)
        raise


def recover_if_damaged(mcap_cli: str, input_path: Path) -> Path:
    """Return input_path, or a recovered copy beside it if mcap doctor fails."""
    check = subprocess.run([mcap_cli, "doctor", str(input_path)], check=False)
    if check.returncode == 0:
        return input_path
    recovered_path = input_path.with_name(f"{input_path.stem}_recovered.mcap")
    if recovered_path.exists():
        raise FileExistsError(f"{recovered_path} already exists")
    print(f"MCAP is damaged; recovering to {recovered_path}")
    recovery = subprocess.run(
        [mcap_cli, "recover", str(input_path), "-o", str(recovered_path)],
        check=False,
    )
    if recovery.returncode not in (0, 3):
        raise RuntimeError(f"mcap recover failed (exit {recovery.returncode})")
    if recovery.returncode == 3:
        print("Warning: recovery discarded some records")
    subprocess.run([mcap_cli, "doctor", str(recovered_path)], check=True)
    return recovered_path


def main():
    parser = argparse.ArgumentParser(prog="hytech", description="HyTech MCAP tool")
    commands = parser.add_subparsers(dest="command", required=True)
    for name, help_text in (
        ("recover", "recover if damaged, then decompress and trim"),
        ("decompress", "decompress the whole recording"),
        ("trim", "split around READY_TO_DRIVE without decompressing"),
        ("strip", "remove camera, lidar, and driverless data"),
    ):
        command = commands.add_parser(name, help=help_text)
        command.add_argument("input", metavar="INPUT.mcap", type=Path)
        if name in ("recover", "decompress"):
            command.add_argument("--video", action="store_true", help="Decode H.264 video frames")
    args = parser.parse_args()

    try:
        input_path = args.input
        if not input_path.is_file():
            raise FileNotFoundError(input_path)

        # check if mcap cli is installed
        mcap_cli = shutil.which("mcap")
        if mcap_cli is None:
            raise RuntimeError("MCAP CLI not found; install it to check and recover files")

        source_path = input_path
        if args.command == "recover":
            source_path = recover_if_damaged(mcap_cli, input_path)

        # decide which time windows to keep
        windows = None
        if args.command in ("recover", "trim"):
            windows = ready_to_drive_windows(source_path)
            if windows is None:
                if args.command == "trim":
                    raise RuntimeError(f"no {CAR_STATES_SCHEMA} messages to trim with")
                print(f"No {CAR_STATES_SCHEMA} messages; keeping the whole recording")
            elif not windows:
                print("Car never reached READY_TO_DRIVE; nothing to write")
                return
        if args.command == "strip":
            outputs = [(input_path.with_name(f"{input_path.stem}_stripped.mcap"), None)]
        elif windows is None:
            outputs = [(input_path.with_name(f"{input_path.stem}_decompressed.mcap"), None)]
        else:
            outputs = [
                (input_path.with_name(f"{input_path.stem}_{index}.mcap"), window)
                for index, window in enumerate(windows)
            ]
        for output_path, _ in outputs:
            if output_path.exists():
                raise FileExistsError(f"{output_path} already exists")

        counts = convert(
            source_path,
            outputs,
            decompress=args.command in ("recover", "decompress"),
            decode_video=getattr(args, "video", False),
            strip=args.command == "strip",
        )

        # check that newly produced mcaps are valid
        for output_path, _ in outputs:
            subprocess.run([mcap_cli, "doctor", str(output_path)], check=True)
    except Exception as exc:
        print(f"{args.command} failed: {exc}")
        raise SystemExit(1)

    for output_path, window in outputs:
        if window is None:
            print(f"Wrote {output_path}")
        else:
            print(f"Wrote {output_path}: {(window[1] - window[0]) / 1e9:.1f} s")
    if args.command == "strip":
        print(
            f"Stripped {counts['stripped']} camera/lidar/driverless messages, "
            f"copied {counts['copied']} other messages"
        )
        return
    print(
        f"Converted {counts['cameras']} camera frames, "
        f"{counts['ranges']} point clouds, {counts['copied']} other messages"
    )


if __name__ == "__main__":
    main()
