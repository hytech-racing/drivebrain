#!/usr/bin/env python3
"""Check capture timestamps on two camera /perception topics in an MCAP."""
import argparse
import statistics
from collections import defaultdict

from mcap.reader import make_reader
from mcap_protobuf.decoder import DecoderFactory


def compare_frames(left, right, tolerance_ns):
    """Match timestamps one-to-one; count every unmatched frame on either side."""
    left, right = sorted(left), sorted(right)
    i = j = 0
    skews = []
    while i < len(left) and j < len(right):
        delta = left[i] - right[j]
        if abs(delta) <= tolerance_ns:
            skews.append(abs(delta))
            i += 1
            j += 1
        elif delta < 0:
            i += 1
        else:
            j += 1
    return skews, len(left) + len(right) - 2 * len(skews)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording")
    parser.add_argument("--tolerance-us", type=float, default=200)
    args = parser.parse_args()
    if not 0 < args.tolerance_us < 10000:
        parser.error("tolerance must be between 0 and 10000 microseconds")
    frames = defaultdict(list)
    with open(args.recording, "rb") as source:
        reader = make_reader(source, decoder_factories=[DecoderFactory()])
        for _, channel, _, image in reader.iter_decoded_messages():
            if channel.topic.startswith("blackfly_") and channel.topic.endswith("/perception"):
                frames[channel.topic].append(image.timestamp.seconds * 1_000_000_000 + image.timestamp.nanos)
    if len(frames) != 2:
        raise SystemExit(f"Expected two camera /perception topics, found {len(frames)}")
    left, right = sorted(frames)
    skews, unmatched = compare_frames(frames[left], frames[right], args.tolerance_us * 1000)
    print(f"{left}: {len(frames[left])} frames; {right}: {len(frames[right])} frames")
    print(f"Matched pairs: {len(skews)}; unmatched frames: {unmatched}")
    if skews:
        print(f"Capture skew: median {statistics.median(skews) / 1000:.1f} us; max {max(skews) / 1000:.1f} us")
    raise SystemExit(0 if skews and unmatched == 0 else 1)


if __name__ == "__main__":
    main()
