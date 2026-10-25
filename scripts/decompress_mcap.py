#!/usr/bin/env python3
"""Convert Drivebrain's compressed camera and Ouster MCAP payloads.

Camera <name>/raw messages become ordinary foxglove.RawImage Bayer BGGR8
messages. Ouster range images and their column timestamps become x, y, z, t
foxglove.PointCloud messages on ouster/points, matching the cloud OusterComms
builds. When a recording has no Ouster calibration message, the metadata
captured in 235.mcap is used. All other messages, plus attachments and
metadata, are copied.

Install Python dependencies with:
    python3 -m pip install -r requirements.txt
Install the MCAP CLI separately, then run:
    ./scripts/decompress_mcap.py recording.mcap

This writes recording_decompressed.mcap next to the input.

The input is never modified. If MCAP recovery is needed, the recovered copy
is kept beside the output. Recovery may discard unreadable records.
"""

from __future__ import annotations

from pathlib import Path
import shutil
import subprocess
import sys

from foxglove_schemas_protobuf.PackedElementField_pb2 import PackedElementField
from foxglove_schemas_protobuf.PointCloud_pb2 import PointCloud
from google.protobuf import descriptor_pb2
from mcap.reader import make_reader
from mcap.writer import Writer
from mcap_protobuf.decoder import DecoderFactory
import numpy as np
from ouster.sdk import core
import zstandard as zstd


CAMERA_ENCODING = "bayer_bggr8"
CALIBRATION_TOPIC = "ouster/calibration"
FALLBACK_CALIBRATION = Path(__file__).with_name("ouster_calibration.json")
RANGE_TOPIC = "ouster/range"
COLUMN_TS_TOPIC = "ouster/column_timestamps"
POINTS_TOPIC = "ouster/points"
MAX_FRAME_BYTES = 512 * 1024 * 1024


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


def column_times(stamps: np.ndarray) -> np.ndarray:
    """Seconds of each column before the last valid column (0 for missing columns), as in OusterComms."""
    end = np.int64(stamps.max())
    times = ((stamps.astype(np.int64) - end).astype(np.float64) * 1e-9).astype(np.float32)
    times[stamps == 0] = 0.0
    return times


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


def pointcloud_schema() -> bytes:
    """Build the MCAP protobuf descriptor set from Foxglove's PointCloud class."""
    files = {}

    def add_file(file):
        if file.name in files:
            return
        for dependency in file.dependencies:
            add_file(dependency)
        proto = descriptor_pb2.FileDescriptorProto()
        file.CopyToProto(proto)
        files[file.name] = proto

    add_file(PointCloud.DESCRIPTOR.file)
    return descriptor_pb2.FileDescriptorSet(file=list(files.values())).SerializeToString()


def convert(input_path: Path, output_path: Path) -> dict[str, int]:
    counts = {"cameras": 0, "ranges": 0, "copied": 0}
    created_output = False
    try:
        with input_path.open("rb") as source:
            reader = make_reader(source)
            decoder_factory = DecoderFactory()

            schema_ids: dict[int, int] = {} # 
            channel_ids: dict[int, int] = {} # link topic to a schema
            column_stamps: dict[tuple[int, int], np.ndarray] = {} # column timestamps by frame stamp
            embedded_calibration = next(reader.iter_messages(topics=[CALIBRATION_TOPIC]), None)
            if embedded_calibration is None:
                lut = calibration_lut(FALLBACK_CALIBRATION.read_text(encoding="utf-8"))
                print(f"No {CALIBRATION_TOPIC} message; using {FALLBACK_CALIBRATION}")
            else:
                calibration_schema, calibration_channel, calibration_message = embedded_calibration
                calibration = decode_proto(
                    decoder_factory, calibration_schema, calibration_channel, calibration_message.data
                )
                if calibration.encoding != "ouster_sensor_metadata_json":
                    raise ValueError("unexpected Ouster calibration encoding")
                lut = calibration_lut(calibration.data.decode("utf-8"))

            with output_path.open("xb") as destination:
                created_output = True
                writer = Writer(destination)
                header = reader.get_header()
                writer.start(profile=header.profile, library="drivebrain-mcap-decompress")

                def output_schema(schema):
                    if schema is None:
                        return 0
                    if schema.id not in schema_ids:
                        schema_ids[schema.id] = writer.register_schema(
                            name=schema.name, encoding=schema.encoding, data=schema.data
                        )
                    return schema_ids[schema.id]

                def output_channel(schema, channel):
                    if channel.id not in channel_ids:
                        channel_ids[channel.id] = writer.register_channel(
                            topic=channel.topic,
                            message_encoding=channel.message_encoding,
                            schema_id=output_schema(schema),
                            metadata=channel.metadata,
                        )
                    return channel_ids[channel.id]

                point_channel_id = None
                for schema, channel, message in reader.iter_messages(log_time_order=False):
                    topic = channel.topic
                    payload = message.data
                    emit_original = True

                    try:
                        if topic == CALIBRATION_TOPIC:
                            calibration = decode_proto(decoder_factory, schema, channel, payload)
                            if calibration.encoding != "ouster_sensor_metadata_json":
                                raise ValueError("unexpected Ouster calibration encoding")
                            lut = calibration_lut(calibration.data.decode("utf-8"))

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

                        elif topic == COLUMN_TS_TOPIC:
                            stamps = decode_proto(decoder_factory, schema, channel, payload)
                            if stamps.encoding != "uint64_ns":
                                raise ValueError(f"unexpected column timestamp encoding {stamps.encoding!r}")
                            column_stamps[(stamps.timestamp.seconds, stamps.timestamp.nanos)] = np.frombuffer(
                                stamps.data, dtype="<u8"
                            )

                        elif topic == RANGE_TOPIC:
                            image = decode_proto(decoder_factory, schema, channel, payload)
                            if image.encoding:
                                raise ValueError(f"unexpected range encoding {image.encoding!r}")
                            if (image.width, image.height) != (lut.w, lut.h):
                                raise ValueError(
                                    f"range size {image.width}x{image.height} differs "
                                    f"from calibration {lut.w}x{lut.h}"
                                )
                            stamps = column_stamps.pop((image.timestamp.seconds, image.timestamp.nanos), None)
                            if stamps is None or stamps.size != image.width:
                                raise ValueError("no matching column timestamps")
                            ranges = decode_ranges(image.data, image.width, image.height)
                            # A zero range means no return. Keep every real return.
                            returns = ranges != 0
                            xyz = lut(ranges)[returns]
                            times = np.broadcast_to(column_times(stamps), ranges.shape)[returns]
                            points = np.ascontiguousarray(np.column_stack((xyz, times)), dtype="<f4")
                            cloud = PointCloud()
                            cloud.timestamp.seconds = image.timestamp.seconds
                            cloud.timestamp.nanos = image.timestamp.nanos
                            cloud.frame_id = image.frame_id or "os_sensor"
                            cloud.pose.orientation.w = 1.0
                            cloud.point_stride = 16
                            for offset, name in enumerate(("x", "y", "z", "t")):
                                field = cloud.fields.add()
                                field.name = name
                                field.offset = offset * 4
                                field.type = PackedElementField.FLOAT32
                            cloud.data = points.tobytes()
                            if point_channel_id is None:
                                point_schema_id = writer.register_schema(
                                    name="foxglove.PointCloud",
                                    encoding="protobuf",
                                    data=pointcloud_schema(),
                                )
                                point_channel_id = writer.register_channel(
                                    topic=POINTS_TOPIC,
                                    message_encoding="protobuf",
                                    schema_id=point_schema_id,
                                )
                            writer.add_message(
                                channel_id=point_channel_id,
                                log_time=message.log_time,
                                publish_time=message.publish_time,
                                sequence=message.sequence,
                                data=cloud.SerializeToString(),
                            )
                            counts["ranges"] += 1
                            emit_original = False
                    except Exception as exc:
                        raise ValueError(f"{topic} at {message.log_time}: {exc}") from exc

                    # copy messages that aren't lidar/camera as is
                    if emit_original:
                        writer.add_message(
                            channel_id=output_channel(schema, channel),
                            log_time=message.log_time,
                            publish_time=message.publish_time,
                            sequence=message.sequence,
                            data=payload,
                        )
                        if topic not in (RANGE_TOPIC,) and not (
                            topic.startswith("blackfly_") and topic.endswith("/raw")
                        ):
                            counts["copied"] += 1

                for attachment in reader.iter_attachments():
                    writer.add_attachment(
                        create_time=attachment.create_time,
                        log_time=attachment.log_time,
                        name=attachment.name,
                        media_type=attachment.media_type,
                        data=attachment.data,
                    )
                for metadata in reader.iter_metadata():
                    writer.add_metadata(name=metadata.name, data=metadata.metadata)
                writer.finish()

        return counts
    except Exception:
        # only delete if newly created 
        if created_output:
            output_path.unlink(missing_ok=True)
        raise


def main():
    if len(sys.argv) != 2:
        print("Usage: decompress_mcap.py INPUT.mcap")
        return
    try:
        # parse input args
        input_path = Path(sys.argv[1])
        if not input_path.is_file():
            raise FileNotFoundError(input_path)
        output_path = input_path.with_name(f"{input_path.stem}_decompressed.mcap")
        if output_path.exists():
            raise FileExistsError(output_path)

        # check if mcap cli is installed
        mcap_cli = shutil.which("mcap")
        if mcap_cli is None:
            raise RuntimeError("MCAP CLI not found; install it to check and recover files")

        # check if the input file is damaged and recover it if necessary
        source_path = input_path
        check = subprocess.run([mcap_cli, "doctor", str(input_path)], check=False)
        if check.returncode != 0:
            recovered_path = input_path.with_name(f"{input_path.stem}_recovered.mcap")
            if recovered_path.exists():
                raise FileExistsError(recovered_path)
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
            source_path = recovered_path

        counts = convert(source_path, output_path)

        # check that newly produced mcap is valid
        subprocess.run([mcap_cli, "doctor", str(output_path)], check=True)
    except Exception as exc:
        print(f"Conversion failed: {exc}")
        return

    print(
        f"Wrote {output_path}: {counts['cameras']} Bayer frames, "
        f"{counts['ranges']} point clouds, {counts['copied']} other messages"
    )


if __name__ == "__main__":
    main()
