#!/usr/bin/env python3
"""Generate bench identity and optional nanopb files from one set of inputs."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ht-proto", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    parser.add_argument("--nanopb", action="store_true")
    args = parser.parse_args()
    bench = Path(__file__).resolve().parent / "proto"
    sources = [args.ht_proto / "base_msgs.proto", args.ht_proto / "hytech_msgs.proto",
               bench / "uart_bench.proto", bench / "uart_bench.options"]
    digest = hashlib.sha256()
    for source in sources:
        digest.update(source.name.encode() + b"\0" + source.read_bytes() + b"\0")
    identity = digest.hexdigest()
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "uart_bench_schema.h").write_text(
        '#pragma once\n#define UART_BENCH_SCHEMA_SHA256 "' + identity + '"\n')
    (args.out / "manifest.json").write_text(json.dumps({
        "version": 1, "schema_sha256": identity,
        "files": {p.name: hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}}, indent=2) + "\n")
    if args.nanopb:
        import nanopb
        generator = Path(nanopb.__file__).parent / "generator" / "nanopb_generator.py"
        command = [sys.executable, str(generator), "-D", str(args.out.resolve()),
                   "-I", str(args.ht_proto.resolve()), "-I", str(bench),
                   "-f", str(bench / "uart_bench.options")]
        # Existing production options bound the other schema's arrays and strings.
        options = args.ht_proto / "hytech_msgs.options"
        production = [sys.executable, str(generator), "-D", str(args.out.resolve()),
                      "-I", str(args.ht_proto.resolve())]
        if options.exists():
            production += ["-f", str(options.resolve())]
        subprocess.run(production + [str(p.resolve()) for p in sources[:2]], check=True)
        subprocess.run(command + [str(bench / "uart_bench.proto")], check=True)
    print(identity)


if __name__ == "__main__":
    main()
