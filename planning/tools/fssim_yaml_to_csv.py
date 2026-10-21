"""Convert an FSSIM track .yaml into a cone CSV (x,y,color), one cone per row.

Colors match dv_msgs::Cones::ConeColor as the planner uses them: left = blue,
right = yellow. There is no header row, same as path.csv.
"""

import argparse
import csv
from pathlib import Path

import yaml

# FSSIM yaml key -> color name
CONE_KEYS = {
    "cones_left": "blue",
    "cones_right": "yellow",
    "cones_orange": "orange_small",
    "cones_orange_big": "orange_big",
}


def convert(yaml_path, csv_path):
    with open(yaml_path) as f:
        track = yaml.safe_load(f)

    rows = [
        (x, y, color)
        for key, color in CONE_KEYS.items()
        for x, y in (track.get(key) or [])
    ]

    with open(csv_path, "w", newline="") as f:
        writer = csv.writer(f)
        for x, y, color in rows:
            writer.writerow([f"{x:.3f}", f"{y:.3f}", color])
    return len(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("yaml_file", type=Path, help="FSSIM track yaml (e.g. FSG.yaml)")
    parser.add_argument("-o", "--output", type=Path, help="output csv (default: <yaml_file>.csv)")
    args = parser.parse_args()

    output = args.output or args.yaml_file.with_suffix(".csv")
    count = convert(args.yaml_file, output)
    print(f"wrote {count} cones to {output}")


if __name__ == "__main__":
    main()
