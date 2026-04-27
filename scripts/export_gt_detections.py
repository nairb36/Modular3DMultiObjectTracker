#!/usr/bin/env python3
"""
Export ground-truth detections from a nuScenes scene to a JSON file.

Usage:
    python3 scripts/export_gt_detections.py \
        --dataroot /workspace/data/nuscenes \
        --version v1.0-mini \
        --scene-index 0 \
        --output results/gt/scene_0000.json
"""

import argparse
import json
import math
import sys

from nuscenes.nuscenes import NuScenes
from pyquaternion import Quaternion


def parse_args():
    p = argparse.ArgumentParser(description="Export GT detections from a nuScenes scene.")
    p.add_argument("--dataroot", default="/workspace/data/nuscenes")
    p.add_argument("--version", default="v1.0-mini")
    p.add_argument("--scene-index", type=int, default=0)
    p.add_argument("--output", default="results/gt/scene_0000.json")
    return p.parse_args()


def yaw_from_quaternion(q):
    """Extract yaw (rotation about z-axis) from a [w, x, y, z] quaternion."""
    quat = Quaternion(q)
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y ** 2 + quat.z ** 2),
    )


def export_scene(nusc, scene):
    frames = []
    sample_token = scene["first_sample_token"]
    frame_id = 0

    while sample_token:
        sample = nusc.get("sample", sample_token)
        ann_tokens = sample["anns"]

        detections = []
        for ann_token in ann_tokens:
            ann = nusc.get("sample_annotation", ann_token)
            rotation = list(ann["rotation"])
            detections.append({
                "instance_token": ann["instance_token"],
                "category_name": ann["category_name"],
                "translation": list(ann["translation"]),
                "size": list(ann["size"]),
                "rotation": rotation,
                "yaw": yaw_from_quaternion(rotation),
            })

        frames.append({
            "frame_id": frame_id,
            "sample_token": sample_token,
            "timestamp": sample["timestamp"],
            "detections": detections,
        })

        sample_token = sample["next"] if sample["next"] else None
        frame_id += 1

    return frames


def main():
    args = parse_args()

    print(f"Loading nuScenes {args.version} from {args.dataroot} ...")
    nusc = NuScenes(version=args.version, dataroot=args.dataroot, verbose=False)

    if args.scene_index < 0 or args.scene_index >= len(nusc.scene):
        sys.exit(
            f"ERROR: scene-index {args.scene_index} out of range "
            f"(dataset has {len(nusc.scene)} scenes)"
        )

    scene = nusc.scene[args.scene_index]
    print(f"Exporting scene {args.scene_index}: {scene['name']}")

    frames = export_scene(nusc, scene)

    with open(args.output, "w") as f:
        json.dump(frames, f, indent=2)

    total_dets = sum(len(fr["detections"]) for fr in frames)
    print(f"Wrote {len(frames)} frames, {total_dets} total detections -> {args.output}")


if __name__ == "__main__":
    main()
