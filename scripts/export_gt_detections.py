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

UNIVERSAL_CATEGORY_MAP = {
    "human.pedestrian.adult": "pedestrian",
    "human.pedestrian.child": "pedestrian",
    "human.pedestrian.construction_worker": "pedestrian",
    "human.pedestrian.personal_mobility": "pedestrian",
    "human.pedestrian.police_officer": "pedestrian",
    "human.pedestrian.stroller": "pedestrian",
    "human.pedestrian.wheelchair": "pedestrian",
    "vehicle.car": "car",
    "vehicle.truck": "truck",
    "vehicle.bus.bendy": "bus",
    "vehicle.bus.rigid": "bus",
    "vehicle.construction": "construction_vehicle",
    "vehicle.motorcycle": "motorcycle",
    "vehicle.bicycle": "bicycle",
    "vehicle.trailer": "trailer",
    "movable_object.barrier": "barrier",
    "movable_object.trafficcone": "traffic_cone",
    "movable_object.debris": "barrier",
    "movable_object.pushable_pullable": "barrier",
    "static_object.bicycle_rack": "barrier",
}


def parse_args():
    p = argparse.ArgumentParser(description="Export GT detections from a nuScenes scene.")
    p.add_argument("--dataroot", default="/workspace/data/nuscenes")
    p.add_argument("--version", default="v1.0-mini")
    p.add_argument("--scene-index", type=int, default=0)
    p.add_argument("--output", default="results/gt/scene_0000.json")
    p.add_argument("--all", action="store_true",
                   help="Export all scenes. Output is treated as a directory.")
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
            raw_cat = ann["category_name"]
            category = UNIVERSAL_CATEGORY_MAP.get(raw_cat, raw_cat)
            detections.append({
                "instance_token": ann["instance_token"],
                "category_name": category,
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


def export_and_write(nusc, scene_index, output_path):
    scene = nusc.scene[scene_index]
    print(f"Exporting scene {scene_index}: {scene['name']}")
    frames = export_scene(nusc, scene)
    with open(output_path, "w") as f:
        json.dump(frames, f, indent=2)
    total_dets = sum(len(fr["detections"]) for fr in frames)
    print(f"  Wrote {len(frames)} frames, {total_dets} detections -> {output_path}")


def main():
    args = parse_args()

    print(f"Loading nuScenes {args.version} from {args.dataroot} ...")
    nusc = NuScenes(version=args.version, dataroot=args.dataroot, verbose=False)

    if args.all:
        import os
        out_dir = args.output
        os.makedirs(out_dir, exist_ok=True)
        for i in range(len(nusc.scene)):
            out_path = os.path.join(out_dir, f"scene_{i:04d}.json")
            export_and_write(nusc, i, out_path)
    else:
        if args.scene_index < 0 or args.scene_index >= len(nusc.scene):
            sys.exit(
                f"ERROR: scene-index {args.scene_index} out of range "
                f"(dataset has {len(nusc.scene)} scenes)"
            )
        export_and_write(nusc, args.scene_index, args.output)


if __name__ == "__main__":
    main()
