#!/usr/bin/env python3
"""
Validate a ground-truth detection JSON file exported by export_gt_detections.py.

Usage:
    python3 scripts/validate_gt_json.py --input results/gt/scene_0000.json
"""

import argparse
import json
import sys


REQUIRED_FRAME_KEYS = {"frame_id", "sample_token", "timestamp", "detections"}
REQUIRED_DET_KEYS = {"instance_token", "category_name", "translation", "size", "rotation", "yaw"}


def parse_args():
    p = argparse.ArgumentParser(description="Validate a GT detection JSON file.")
    p.add_argument("--input", required=True, help="Path to the JSON file to validate")
    return p.parse_args()


def main():
    args = parse_args()

    try:
        with open(args.input) as f:
            data = json.load(f)
    except FileNotFoundError:
        sys.exit(f"ERROR: file not found: {args.input}")
    except json.JSONDecodeError as e:
        sys.exit(f"ERROR: invalid JSON: {e}")

    if not isinstance(data, list):
        sys.exit(f"ERROR: top-level JSON must be a list, got {type(data).__name__}")

    total_dets = 0
    for i, frame in enumerate(data):
        missing = REQUIRED_FRAME_KEYS - set(frame.keys())
        if missing:
            sys.exit(f"ERROR: frame {i} missing keys: {missing}")

        if not isinstance(frame["detections"], list):
            sys.exit(f"ERROR: frame {i} 'detections' is not a list")

        for j, det in enumerate(frame["detections"]):
            missing_det = REQUIRED_DET_KEYS - set(det.keys())
            if missing_det:
                sys.exit(f"ERROR: frame {i}, detection {j} missing keys: {missing_det}")
            total_dets += 1

    print(f"Validation PASSED")
    print(f"  File   : {args.input}")
    print(f"  Frames : {len(data)}")
    print(f"  Total detections: {total_dets}")


if __name__ == "__main__":
    main()
