#!/usr/bin/env python3
"""
Export a nuScenes scene into a self-contained scene JSON file.

Produces our custom schema: scene-level calibration, per-frame ego pose,
sensor data file paths, and ground-truth annotations.

Usage:
    python3 scripts/export_scene.py \
        --dataroot /workspace/data/nuscenes \
        --version v1.0-mini \
        --scene-index 0 \
        --output results/scenes/scene_0000.json

    # Export all scenes at once:
    python3 scripts/export_scene.py \
        --dataroot /workspace/data/nuscenes \
        --version v1.0-mini \
        --all \
        --output-dir results/scenes
"""

import argparse
import json
import math
import sys
from pathlib import Path

from nuscenes.nuscenes import NuScenes
from pyquaternion import Quaternion


def parse_args():
    p = argparse.ArgumentParser(description="Export nuScenes scene to custom scene JSON.")
    p.add_argument("--dataroot", default="/workspace/data/nuscenes")
    p.add_argument("--version", default="v1.0-mini")

    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument("--scene-index", type=int, help="Export a single scene by index")
    group.add_argument("--all", action="store_true", help="Export all scenes")

    p.add_argument("--output", help="Output path for single scene export")
    p.add_argument("--output-dir", default="results/scenes", help="Output directory for --all mode")
    return p.parse_args()


def yaw_from_quaternion(q):
    quat = Quaternion(q)
    return math.atan2(
        2.0 * (quat.w * quat.z + quat.x * quat.y),
        1.0 - 2.0 * (quat.y ** 2 + quat.z ** 2),
    )


def build_calibration(nusc, scene):
    """Build scene-level calibration from the first sample's sensor data."""
    first_sample = nusc.get("sample", scene["first_sample_token"])
    calibration = {}

    for channel, sd_token in first_sample["data"].items():
        sd = nusc.get("sample_data", sd_token)
        cs = nusc.get("calibrated_sensor", sd["calibrated_sensor_token"])
        sensor = nusc.get("sensor", cs["sensor_token"])

        entry = {
            "extrinsic": {
                "translation": list(cs["translation"]),
                "rotation": list(cs["rotation"]),
            }
        }

        if cs["camera_intrinsic"]:
            entry["intrinsic"] = cs["camera_intrinsic"]

        calibration[sensor["channel"]] = entry

    return calibration


def get_lidar_sweeps(nusc, lidar_sd, max_sweeps=9):
    """Collect up to max_sweeps previous non-keyframe lidar scans."""
    sweeps = []
    sd = nusc.get("sample_data", lidar_sd["prev"]) if lidar_sd["prev"] else None
    while sd is not None and len(sweeps) < max_sweeps:
        ego = nusc.get("ego_pose", sd["ego_pose_token"])
        sweeps.append({
            "file": sd["filename"],
            "timestamp": sd["timestamp"],
            "ego_pose": {
                "translation": list(ego["translation"]),
                "rotation": list(ego["rotation"]),
            },
        })
        sd = nusc.get("sample_data", sd["prev"]) if sd["prev"] else None
    return sweeps


def export_frame(nusc, sample, frame_id):
    """Export a single sample (keyframe) into our frame schema."""
    # Use LIDAR_TOP ego_pose as the reference pose for the frame
    lidar_sd = nusc.get("sample_data", sample["data"]["LIDAR_TOP"])
    ego = nusc.get("ego_pose", lidar_sd["ego_pose_token"])

    # Sensor data: file paths + metadata per channel
    sensor_data = {}
    for channel, sd_token in sample["data"].items():
        sd = nusc.get("sample_data", sd_token)
        entry = {"file": sd["filename"]}
        if sd["height"] > 0:
            entry["width"] = sd["width"]
            entry["height"] = sd["height"]
        if channel == "LIDAR_TOP":
            entry["sweeps"] = get_lidar_sweeps(nusc, sd)
        sensor_data[channel] = entry

    # GT annotations
    annotations = []
    for ann_token in sample["anns"]:
        ann = nusc.get("sample_annotation", ann_token)
        rotation = list(ann["rotation"])
        annotations.append({
            "instance_token": ann["instance_token"],
            "category_name": ann["category_name"],
            "translation": list(ann["translation"]),
            "size": list(ann["size"]),
            "rotation": rotation,
            "yaw": yaw_from_quaternion(rotation),
        })

    return {
        "frame_id": frame_id,
        "sample_token": sample["token"],
        "timestamp": sample["timestamp"],
        "ego_pose": {
            "translation": list(ego["translation"]),
            "rotation": list(ego["rotation"]),
        },
        "sensor_data": sensor_data,
        "annotations": annotations,
    }


def export_scene(nusc, scene):
    calibration = build_calibration(nusc, scene)

    frames = []
    sample_token = scene["first_sample_token"]
    frame_id = 0

    while sample_token:
        sample = nusc.get("sample", sample_token)
        frames.append(export_frame(nusc, sample, frame_id))
        sample_token = sample["next"] if sample["next"] else None
        frame_id += 1

    return {
        "scene_name": scene["name"],
        "description": scene["description"],
        "log_token": scene["log_token"],
        "num_frames": len(frames),
        "calibration": calibration,
        "frames": frames,
    }


def write_scene(scene_data, output_path):
    Path(output_path).parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(scene_data, f, indent=2)

    total_anns = sum(len(fr["annotations"]) for fr in scene_data["frames"])
    print(
        f"  {scene_data['scene_name']}: "
        f"{scene_data['num_frames']} frames, "
        f"{total_anns} annotations -> {output_path}"
    )


def main():
    args = parse_args()

    print(f"Loading nuScenes {args.version} from {args.dataroot} ...")
    nusc = NuScenes(version=args.version, dataroot=args.dataroot, verbose=False)

    if args.all:
        for i, scene in enumerate(nusc.scene):
            scene_data = export_scene(nusc, scene)
            output_path = Path(args.output_dir) / f"scene_{i:04d}.json"
            write_scene(scene_data, output_path)
        print(f"\nExported {len(nusc.scene)} scenes to {args.output_dir}")
    else:
        if args.scene_index < 0 or args.scene_index >= len(nusc.scene):
            sys.exit(
                f"ERROR: scene-index {args.scene_index} out of range "
                f"(dataset has {len(nusc.scene)} scenes)"
            )
        scene = nusc.scene[args.scene_index]
        scene_data = export_scene(nusc, scene)
        output_path = args.output or f"results/scenes/scene_{args.scene_index:04d}.json"
        write_scene(scene_data, output_path)


if __name__ == "__main__":
    main()
