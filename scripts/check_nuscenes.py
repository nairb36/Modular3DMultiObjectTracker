#!/usr/bin/env python3
"""
Sanity-check script for a nuScenes dataset mount.

Usage:
    python3 scripts/check_nuscenes.py --dataroot /datasets/nuscenes --version v1.0-mini
"""

import argparse
import os
import sys


def parse_args():
    p = argparse.ArgumentParser(description="Verify a nuScenes dataset installation.")
    p.add_argument(
        "--dataroot",
        default="/data/nuscenes",
        help="Root directory of the nuScenes dataset (default: /data/nuscenes)",
    )
    p.add_argument(
        "--version",
        default="v1.0-mini",
        help="Dataset version string (default: v1.0-mini)",
    )
    return p.parse_args()


def check_path(path: str, label: str) -> bool:
    exists = os.path.exists(path)
    status = "OK" if exists else "MISSING"
    print(f"  [{status}] {label}: {path}")
    return exists


def main():
    args = parse_args()

    # --- Pre-flight checks -----------------------------------------------
    if not os.path.isdir(args.dataroot):
        sys.exit(
            f"ERROR: dataroot not found: '{args.dataroot}'\n"
            "  Make sure the dataset volume is mounted and the path is correct."
        )

    version_dir = os.path.join(args.dataroot, args.version)
    if not os.path.isdir(version_dir):
        available = [
            d for d in os.listdir(args.dataroot)
            if os.path.isdir(os.path.join(args.dataroot, d))
        ]
        sys.exit(
            f"ERROR: version directory not found: '{version_dir}'\n"
            f"  Available directories under {args.dataroot}: {available or ['(none)']}"
        )

    # --- Load nuScenes ----------------------------------------------------
    try:
        from nuscenes.nuscenes import NuScenes
    except ImportError:
        sys.exit(
            "ERROR: nuscenes-devkit is not installed.\n"
            "  Install it with: pip install nuscenes-devkit"
        )

    print(f"\nLoading nuScenes {args.version} from {args.dataroot} …")
    nusc = NuScenes(version=args.version, dataroot=args.dataroot, verbose=False)

    # --- Top-level counts -------------------------------------------------
    print("\n=== Dataset summary ===")
    print(f"  Scenes            : {len(nusc.scene)}")
    print(f"  Samples           : {len(nusc.sample)}")
    print(f"  Sample data       : {len(nusc.sample_data)}")
    print(f"  Calibrated sensors: {len(nusc.calibrated_sensor)}")
    print(f"  Ego poses         : {len(nusc.ego_pose)}")

    # --- First scene ------------------------------------------------------
    first_scene = nusc.scene[0]
    print(f"\n=== First scene ===")
    print(f"  Name       : {first_scene['name']}")
    print(f"  Description: {first_scene['description']}")
    print(f"  Token      : {first_scene['token']}")

    # --- First sample in that scene ---------------------------------------
    first_sample = nusc.get("sample", first_scene["first_sample_token"])
    print(f"\n=== First sample ===")
    print(f"  Token    : {first_sample['token']}")
    print(f"  Timestamp: {first_sample['timestamp']} µs")

    # Collect sensor channels present in this sample
    channels = list(first_sample["data"].keys())
    print(f"  Channels : {', '.join(sorted(channels))}")

    # --- File existence checks for first sample ---------------------------
    print(f"\n=== File existence check (first sample) ===")

    all_ok = True

    # LIDAR_TOP
    if "LIDAR_TOP" in first_sample["data"]:
        sd = nusc.get("sample_data", first_sample["data"]["LIDAR_TOP"])
        path = os.path.join(args.dataroot, sd["filename"])
        ok = check_path(path, "LIDAR_TOP")
        all_ok = all_ok and ok
    else:
        print("  [SKIP] LIDAR_TOP not present in this sample")

    # All cameras (channels whose names start with "CAM_")
    camera_channels = sorted(ch for ch in channels if ch.startswith("CAM_"))
    for cam in camera_channels:
        sd = nusc.get("sample_data", first_sample["data"][cam])
        path = os.path.join(args.dataroot, sd["filename"])
        ok = check_path(path, cam)
        all_ok = all_ok and ok

    # --- Final verdict ----------------------------------------------------
    print()
    if all_ok:
        print("All file checks passed. Dataset looks healthy.")
    else:
        print("WARNING: Some files are missing. Check the paths above.")
        sys.exit(1)


if __name__ == "__main__":
    main()
