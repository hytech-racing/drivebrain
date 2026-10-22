#!/usr/bin/env python3
r"""Extract selected stereo frames and inspect chessboard-shape residuals.

The timestamps below are foxglove.RawImage.timestamp values, not MCAP log times.
The recording contains /perception channels (RawImage, bgr8), not /raw channels.

Run with the supplied board and lens estimates:
  python3 scripts/check_chessboard_floppiness.py recovered.mcap

Defaults: 14x14 inner corners; 15 mm measured board-square spacing;
885 px for the wider 8.5 mm view on camera 748 and
1667 px for the narrower 16 mm view on camera 1609. The focal estimates
assume 4.8 um native sensor pixels and the 2x downsampling to 640x512 in
Drivebrain. They seed the calibration; they are not its fitted result.
Override spacing if a different board is used. Add
--extract-only to write the images and pairing manifest without mrcal/mrgingham.
The early/late center-warp difference is an estimate of shape change under
mrcal's two-parameter parabolic board model, not a direct physical measurement.
"""

from __future__ import annotations

import argparse
from collections import deque
import csv
import json
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import zlib

FIRST_TOPIC = "blackfly_26191748/perception"
SECOND_TOPIC = "blackfly_26291609/perception"
NSEC = 1_000_000_000
DEFAULT_CORNER_SPACING_M = 0.015
DEFAULT_FOCALS_PX = "885,1667"
SELECTED = (
    (1792391796, 839176256),
    (1792391797, 42999360),
    (1792391797, 635217600),
    (1792391798, 171089184),
    (1792391798, 639252576),
    (1792391799, 299309696),
    (1792391799, 499409632),
    (1792391800, 703187200),
    (1792391801, 635366816),
    (1792391801, 707034080),
    (1792391801, 979004032),
    (1792391802, 303303264),
    (1792391802, 507040896),
    (1792391802, 903389952),
    (1792391803, 439055264),
    (1792391804, 103223872),
    (1792391804, 507065408),
    (1792391804, 907065824),
    (1792391805, 311072384),
    (1792391805, 710970624),
    (1792391806, 43042912),
    (1792391806, 303252288),
    (1792391806, 707187040),
    (1792391806, 982901664),
    (1792391807, 439149120),
    (1792391808, 239150496),
    (1792391809, 103273312),
    (1792391811, 503310912),
    (1792391813, 111843136),
    (1792391815, 116451232),
    (1792391816, 43040960),
    (1792391823, 182959552),
    (1792391823, 511068224),
    (1792391823, 775201600),
    (1792391824, 582951040),
    (1792391825, 983105056),
    (1792391827, 979121152),
    (1792391828, 975295808),
    (1792391832, 383083840),
)


def timestamp_ns(image) -> int:
    return image.timestamp.seconds * NSEC + image.timestamp.nanos


def write_png(path: Path, image) -> None:
    """Write bgr8 Foxglove RawImage as RGB PNG without image libraries."""
    if image.encoding.lower() != "bgr8":
        raise ValueError(f"{path}: expected bgr8, got {image.encoding!r}")
    width, height = image.width, image.height
    row_bytes = width * 3
    step = image.step or row_bytes
    if width <= 0 or height <= 0 or step < row_bytes or len(image.data) < step * height:
        raise ValueError(f"{path}: invalid image size, step or payload")
    rows = bytearray()
    for y in range(height):
        bgr = image.data[y * step : y * step + row_bytes]
        rgb = bytearray(row_bytes)
        rgb[0::3], rgb[1::3], rgb[2::3] = bgr[2::3], bgr[1::3], bgr[0::3]
        rows.append(0)  # PNG filter type 0
        rows.extend(rgb)

    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))

    with path.open("wb") as out:
        out.write(b"\x89PNG\r\n\x1a\n")
        out.write(chunk(b"IHDR", struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)))
        out.write(chunk(b"IDAT", zlib.compress(rows, level=3)))
        out.write(chunk(b"IEND", b""))


def extract(mcap_path: Path, output_dir: Path) -> list[tuple[int, int]]:
    """Match each exact 748 timestamp to the first later 1609 image timestamp."""
    try:
        from mcap.reader import make_reader
        from mcap_protobuf.decoder import DecoderFactory
    except ImportError as exc:
        raise RuntimeError("install mcap and mcap-protobuf-support first") from exc
    requested = {sec * NSEC + nsec: i for i, (sec, nsec) in enumerate(SELECTED)}
    if len(requested) != len(SELECTED):
        raise ValueError("duplicate requested timestamps")
    pending = deque()
    first_images = {}
    matches = {}
    decoder_factory = DecoderFactory()

    # MCAP indexes log time, while the requested values are embedded image time.
    # The 2 s margin covers the small transport/logging delay in this recording.
    with mcap_path.open("rb") as stream:
        reader = make_reader(stream)
        for schema, channel, message in reader.iter_messages(
            topics=[FIRST_TOPIC, SECOND_TOPIC],
            starting_at=min(requested) - 2 * NSEC,
            ending_before=max(requested) + 2 * NSEC,
        ):
            if schema is None or channel.message_encoding != "protobuf":
                raise ValueError(f"{channel.topic}: expected protobuf RawImage")
            decoder = decoder_factory.decoder_for(channel.message_encoding, schema)
            image = decoder(message.data)
            ts = timestamp_ns(image)
            if channel.topic == FIRST_TOPIC:
                if ts in requested:
                    if ts in first_images:
                        raise ValueError(f"duplicate 748 image timestamp {ts}")
                    first_images[ts] = image
                    pending.append(ts)
            else:
                while pending and pending[0] < ts:
                    first_ts = pending.popleft()
                    matches[first_ts] = (ts, image)
            if len(matches) == len(requested):
                break

    missing = sorted(set(requested) - set(first_images))
    unpaired = sorted(set(first_images) - set(matches))
    if missing or unpaired:
        raise ValueError(f"missing 748 timestamps: {missing}; without later 1609 image: {unpaired}")

    output_dir.mkdir(parents=True, exist_ok=True)
    manifest = output_dir / "pairs.csv"
    gaps = []
    with manifest.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("frame", "camera0_timestamp_ns", "camera1_timestamp_ns", "gap_ms", "camera0_image", "camera1_image"))
        for first_ts, index in sorted(requested.items(), key=lambda item: item[1]):
            second_ts, second_image = matches[first_ts]
            first_path = output_dir / f"frame{index:03d}-camera0.png"
            second_path = output_dir / f"frame{index:03d}-camera1.png"
            write_png(first_path, first_images[first_ts])
            write_png(second_path, second_image)
            gap_ms = (second_ts - first_ts) / 1e6
            gaps.append(gap_ms)
            writer.writerow((index, first_ts, second_ts, f"{gap_ms:.6f}", first_path.name, second_path.name))
    print(f"Extracted {len(matches)} pairs to {output_dir}")
    print(f"Pairing gaps: {min(gaps):.1f}–{max(gaps):.1f} ms (see {manifest})")
    return [(first_ts, matches[first_ts][0]) for first_ts in requested]


def mrcal_command(command: list[str]) -> list[str]:
    """Run mrcal's Python scripts with this environment's interpreter."""
    executable = shutil.which(command[0])
    if executable is None:
        raise RuntimeError(f"{command[0]} is not on PATH")
    return [sys.executable, executable, *command[1:]]


def write_opencv_corners(images: list[Path], cache: Path, width: int, height: int) -> None:
    """Write the vnlog corner cache accepted by mrcal when mrgingham is absent."""
    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError("mrgingham is unavailable; install opencv-python-headless for corner detection") from exc
    detected = 0
    with cache.open("w") as stream:
        stream.write("# filename x y level\n")
        for image in images:
            gray = cv2.imread(str(image), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                raise RuntimeError(f"cannot read {image}")
            found, corners = cv2.findChessboardCornersSB(
                gray, (width, height), cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
            )
            if not found:
                stream.write(f"{image} - - -\n")
                continue
            detected += 1
            for x, y in corners.reshape(-1, 2):
                stream.write(f"{image} {x:.6f} {y:.6f} 0\n")
    print(f"OpenCV found the {width}x{height} board in {detected}/{len(images)} images; cache: {cache}", flush=True)
    if detected < 3:
        raise RuntimeError(f"too few board detections in {cache}")


def copy_corner_subset(source: Path, destination: Path, directory: Path, images: list[Path]) -> None:
    names = {image.name for image in images}
    with source.open() as infile, destination.open("w") as outfile:
        outfile.write("# filename x y level\n")
        for line in infile:
            if line.startswith("#"):
                continue
            fields = line.split()
            if fields and Path(fields[0]).name in names:
                fields[0] = str(directory / Path(fields[0]).name)
                outfile.write(" ".join(fields) + "\n")


def run_calibration(args, output_dir: Path) -> None:
    if not shutil.which("mrcal-calibrate-cameras"):
        raise RuntimeError("mrcal-calibrate-cameras is not on PATH; images are extracted")
    if not shutil.which("mrcal-show-residuals-board-observation"):
        raise RuntimeError("mrcal-show-residuals-board-observation is not on PATH; images are extracted")

    summary = {}
    use_opencv = not shutil.which("mrgingham")
    # Solve each camera independently: successive frames are not synchronized,
    # so a joint solve would turn board motion into apparent shape residuals.
    for camera in (0, 1):
        camera_dir = output_dir / f"camera{camera}"
        camera_dir.mkdir(exist_ok=True)
        images = sorted(output_dir.glob(f"frame*-camera{camera}.png"))
        corners_cache = camera_dir / "corners-png.vnl"
        if use_opencv and not corners_cache.exists():
            write_opencv_corners(images, corners_cache, args.object_width_n, args.object_height_n)
        common = [
            "mrcal-calibrate-cameras",
            "--object-spacing", str(args.object_spacing),
            "--object-width-n", str(args.object_width_n),
            "--object-height-n", str(args.object_height_n),
        ]
        cmd = common + [
            "--lensmodel", args.lensmodels[camera],
            "--focal", str(args.focals[camera]),
            "--outdir", str(camera_dir),
            "--corners-cache", str(corners_cache),
            str(output_dir / f"frame*-camera{camera}.png"),
        ]
        print(f"Calibrating camera {camera}: {('748', '1609')[camera]}", flush=True)
        with (camera_dir / "calibration.log").open("w") as log:
            result = subprocess.run(mrcal_command(cmd), stdout=log, stderr=subprocess.STDOUT, check=False)
        if result.returncode:
            raise RuntimeError(f"camera {camera} calibration failed; see {camera_dir / 'calibration.log'}")
        models = sorted(camera_dir.glob("*.cameramodel"))
        if len(models) != 1:
            raise RuntimeError(f"expected one model in {camera_dir}, found {len(models)}")
        print((camera_dir / "calibration.log").read_text(errors="replace")[-1500:])
        plots = camera_dir / "worst_residuals"
        plots.mkdir(exist_ok=True)
        for rank in range(min(args.worst, len(SELECTED))):
            plot = plots / f"worst-{rank:02d}.svg"
            subprocess.run(mrcal_command([
                "mrcal-show-residuals-board-observation",
                "--from-worst", "--vectorscale", str(args.vectorscale),
                "--circlescale", "0.5", "--hardcopy", str(plot),
                str(models[0]), str(rank),
            ]), check=True)
        print(f"Wrote {min(args.worst, len(SELECTED))} residual plots to {plots}")

        # mrcal represents the constant board deformation with two center
        # deflections. Refit early/late observations with the full-data lens
        # intrinsics fixed, then compare those deflections in millimeters.
        half_models = []
        for label, subset in (("early", images[:len(images) // 2]),
                              ("late", images[len(images) // 2:])):
            half_dir = camera_dir / label
            half_dir.mkdir(exist_ok=True)
            for image in subset:
                link = half_dir / image.name
                if not link.exists():
                    link.symlink_to(image.resolve())
            half_cache = half_dir / "corners-png.vnl"
            if use_opencv and not half_cache.exists():
                copy_corner_subset(corners_cache, half_cache, half_dir, subset)
            half_cmd = common + [
                "--seed", str(models[0]),
                "--skip-intrinsics-solve",
                "--outdir", str(half_dir),
                "--corners-cache", str(half_cache),
                str(half_dir / f"frame*-camera{camera}.png"),
            ]
            with (half_dir / "calibration.log").open("w") as log:
                result = subprocess.run(mrcal_command(half_cmd), stdout=log, stderr=subprocess.STDOUT, check=False)
            if result.returncode:
                raise RuntimeError(f"{label} camera {camera} solve failed; see {half_dir / 'calibration.log'}")
            found = sorted(half_dir.glob("*.cameramodel"))
            if len(found) != 1:
                raise RuntimeError(f"expected one {label} model in {half_dir}")
            half_models.append(found[0])

        try:
            import mrcal
        except ImportError as exc:
            raise RuntimeError("mrcal Python package is needed to read board warp values") from exc

        def center_warp_mm(path: Path) -> list[float]:
            inputs = mrcal.cameramodel(str(path)).optimization_inputs()
            if inputs is None or "calobject_warp" not in inputs:
                raise RuntimeError(f"{path} does not contain board warp results")
            return [float(value) * 1000 for value in inputs["calobject_warp"]]

        full = center_warp_mm(models[0])
        early, late = [center_warp_mm(path) for path in half_models]
        delta = [b - a for a, b in zip(early, late)]
        summary[("748", "1609")[camera]] = {
            "full_center_warp_mm": full,
            "early_center_warp_mm": early,
            "late_center_warp_mm": late,
            "late_minus_early_mm": delta,
            "late_minus_early_magnitude_mm": sum(v * v for v in delta) ** 0.5,
        }
        print(f"Camera {('748', '1609')[camera]} center-warp change: "
              f"({delta[0]:+.3f}, {delta[1]:+.3f}) mm")

    (output_dir / "warp_summary.json").write_text(json.dumps(summary, indent=2) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("mcap", type=Path)
    parser.add_argument("--output", type=Path, default=Path("chessboard_floppiness"))
    parser.add_argument("--extract-only", action="store_true")
    parser.add_argument("--object-width-n", type=int, default=14, help="horizontal inner corners (default: 14)")
    parser.add_argument("--object-height-n", type=int, default=14, help="vertical inner corners (default: 14)")
    parser.add_argument("--object-spacing", type=float, default=DEFAULT_CORNER_SPACING_M,
                        help="adjacent-corner spacing in meters (default: %.3f)" % DEFAULT_CORNER_SPACING_M)
    parser.add_argument("--focal", default=DEFAULT_FOCALS_PX,
                        help="one estimate for both cameras, or 748,1609 estimates in pixels at 640x512 (default: %s)" % DEFAULT_FOCALS_PX)
    parser.add_argument("--lensmodel", default="LENSMODEL_OPENCV8",
                        help="one model for both cameras, or two comma-separated models")
    parser.add_argument("--worst", type=int, default=10, help="worst residual plots per camera")
    parser.add_argument("--vectorscale", type=float, default=100.0)
    args = parser.parse_args()
    if not args.extract_only:
        if args.object_width_n < 2 or args.object_height_n < 2 or args.object_spacing <= 0:
            parser.error("board dimensions must be >=2 and spacing must be positive")
        try:
            focals = [float(value) for value in args.focal.split(",")]
        except ValueError:
            parser.error("--focal must be one positive number or two comma-separated positive numbers")
        if len(focals) not in (1, 2) or any(value <= 0 for value in focals):
            parser.error("--focal must be one positive number or two comma-separated positive numbers")
        args.focals = focals * 2 if len(focals) == 1 else focals
    if args.worst < 0 or args.vectorscale <= 0:
        parser.error("--worst must be nonnegative and --vectorscale must be positive")
    lensmodels = args.lensmodel.split(",")
    if len(lensmodels) not in (1, 2) or any(not value.startswith("LENSMODEL_") for value in lensmodels):
        parser.error("--lensmodel must name one or two comma-separated LENSMODEL_* values")
    args.lensmodels = lensmodels * 2 if len(lensmodels) == 1 else lensmodels
    try:
        extract(args.mcap, args.output)
        if not args.extract_only:
            run_calibration(args, args.output)
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
