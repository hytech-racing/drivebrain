#!/usr/bin/env python3
"""Restore H.264 camera frames to upright, full-resolution bgr8 RawImages.

Usage: python3 scripts/decompress_video_mcap.py INPUT.mcap
Install requirements.txt and the MCAP CLI first. Writes INPUT_decompressed.mcap,
including the existing lidar decompression. H.264 is lossy; Bayer samples cannot
be recovered. Legacy Bayer recordings retain the original converter's behavior.
"""
from decompress_mcap import main

if __name__ == "__main__":
    main(decode_video=True)
