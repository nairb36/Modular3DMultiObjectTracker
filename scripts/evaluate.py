#!/usr/bin/env python3
"""
Evaluate MOT tracker results using the nuScenes tracking evaluation.

Converts tracker output to nuScenes submission format and runs the full
evaluation pipeline (AMOTA, AMOTP, MOTA, MOTP, etc.).

Usage:
    python3 scripts/evaluate.py results/tracking/scene_0000_results_20260514_013812.json
    python3 scripts/evaluate.py results/tracking/scene_*_results_*.json
"""

import argparse
import json
import math
import os
import re
import sys
from collections import defaultdict

from nuscenes.nuscenes import NuScenes
from nuscenes.eval.common.config import config_factory
from nuscenes.eval.tracking.evaluate import TrackingEval
from nuscenes.utils.splits import create_splits_scenes


CATEGORY_TO_TRACKING_NAME = {
    'vehicle.bicycle': 'bicycle',
    'vehicle.bus.bendy': 'bus',
    'vehicle.bus.rigid': 'bus',
    'vehicle.car': 'car',
    'vehicle.motorcycle': 'motorcycle',
    'human.pedestrian.adult': 'pedestrian',
    'human.pedestrian.child': 'pedestrian',
    'human.pedestrian.construction_worker': 'pedestrian',
    'human.pedestrian.police_officer': 'pedestrian',
    'vehicle.trailer': 'trailer',
    'vehicle.truck': 'truck',
}


def yaw_to_quaternion(yaw):
    """Convert yaw (rotation about z-axis) to [w, x, y, z] quaternion."""
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def parse_scene_index(filename):
    """Extract scene index from filename like 'scene_0003_results_*.json'."""
    match = re.search(r'scene_(\d+)', os.path.basename(filename))
    if match:
        return int(match.group(1))
    return None


def convert_tracker_results(tracker_path, gt_scene_path):
    """
    Convert one scene's tracker output to nuScenes submission format.
    Returns dict {sample_token: [box_dict, ...]}.
    """
    with open(tracker_path) as f:
        tracker_data = json.load(f)
    with open(gt_scene_path) as f:
        gt_data = json.load(f)

    frame_to_sample_token = {frame['frame_id']: frame['sample_token'] for frame in gt_data}

    results = {}
    for frame in tracker_data:
        sample_token = frame_to_sample_token[frame['frame_id']]
        boxes = []
        for track in frame['tracks']:
            tracking_name = CATEGORY_TO_TRACKING_NAME.get(track['category_name'])
            if tracking_name is None:
                continue
            boxes.append({
                'sample_token': sample_token,
                'translation': track['translation'],
                'size': track['size'],
                'rotation': yaw_to_quaternion(track['yaw']),
                'velocity': [0.0, 0.0],
                'tracking_id': str(track['id']),
                'tracking_name': tracking_name,
                'tracking_score': track.get('tracking_score', 1.0),
            })
        results[sample_token] = boxes

    return results


def get_all_sample_tokens_for_split(nusc, eval_split):
    """Return all sample_tokens belonging to scenes in the given split."""
    splits = create_splits_scenes()
    split_scene_names = set(splits[eval_split])
    tokens = []
    for scene in nusc.scene:
        if scene['name'] not in split_scene_names:
            continue
        sample_token = scene['first_sample_token']
        while sample_token:
            tokens.append(sample_token)
            sample = nusc.get('sample', sample_token)
            sample_token = sample['next'] if sample['next'] != '' else None
    return tokens


def parse_args():
    p = argparse.ArgumentParser(description='Evaluate MOT tracker results against nuScenes.')
    p.add_argument('result_path', help='Run directory (e.g. results/tracking/20260515_232556) or individual JSON file(s)',
                   nargs='+')
    p.add_argument('--dataroot', default='/workspace/data/nuscenes')
    p.add_argument('--version', default='v1.0-mini')
    p.add_argument('--gt-dir', default=None,
                   help='Directory containing GT scene JSONs (default: results/gt relative to project root)')
    p.add_argument('--output-dir', default=None,
                   help='Directory for evaluation output (default: results/eval relative to project root)')
    args = p.parse_args()

    # If a single directory is given, expand to all scene JSON files inside it
    if len(args.result_path) == 1 and os.path.isdir(args.result_path[0]):
        run_dir = args.result_path[0]
        args.result_files = sorted(
            os.path.join(run_dir, f) for f in os.listdir(run_dir)
            if f.startswith('scene_') and f.endswith('.json')
        )
    else:
        args.result_files = args.result_path

    return args


def main():
    args = parse_args()

    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    gt_dir = args.gt_dir or os.path.join(project_root, 'results', 'gt')
    output_dir = args.output_dir or os.path.join(project_root, 'results', 'eval')

    print(f'Loading nuScenes {args.version} from {args.dataroot} ...')
    nusc = NuScenes(version=args.version, dataroot=args.dataroot, verbose=False)

    index_to_scene_name = {i: scene['name'] for i, scene in enumerate(nusc.scene)}

    splits = create_splits_scenes()
    scene_name_to_split = {}
    for split_name in ['mini_val', 'mini_train']:
        for scene_name in splits[split_name]:
            scene_name_to_split[scene_name] = split_name

    # Group input files by eval split
    split_files = defaultdict(list)
    for result_file in args.result_files:
        scene_idx = parse_scene_index(result_file)
        if scene_idx is None:
            print(f'WARNING: Cannot parse scene index from {result_file!r}, skipping')
            continue
        scene_name = index_to_scene_name.get(scene_idx)
        if scene_name is None:
            print(f'WARNING: Scene index {scene_idx} not in dataset, skipping')
            continue
        eval_split = scene_name_to_split.get(scene_name)
        if eval_split is None:
            print(f'WARNING: Scene {scene_name} not in any eval split, skipping')
            continue
        split_files[eval_split].append((scene_idx, scene_name, result_file))

    if not split_files:
        sys.exit('ERROR: No valid result files to evaluate.')

    # Evaluate each split
    for eval_split, scene_entries in split_files.items():
        split_scene_names = set(splits[eval_split])
        covered = {name for _, name, _ in scene_entries}
        missing = split_scene_names - covered
        if missing:
            print(f'\nWARNING: Split {eval_split!r} has {len(split_scene_names)} scenes '
                  f'but only {len(covered)} have results.')
            print(f'  Missing: {sorted(missing)}')
            print(f'  Missing scenes will have empty predictions (all FN).\n')

        # Convert and merge all scenes
        merged_results = {}
        for scene_idx, scene_name, result_file in scene_entries:
            gt_path = os.path.join(gt_dir, f'scene_{scene_idx:04d}.json')
            if not os.path.exists(gt_path):
                sys.exit(f'ERROR: GT file not found: {gt_path}')
            print(f'  Converting {os.path.basename(result_file)} ({scene_name})')
            scene_results = convert_tracker_results(result_file, gt_path)
            merged_results.update(scene_results)

        # Fill empty predictions for all sample_tokens in the split
        for token in get_all_sample_tokens_for_split(nusc, eval_split):
            if token not in merged_results:
                merged_results[token] = []

        submission = {
            'meta': {
                'use_camera': False,
                'use_lidar': True,
                'use_radar': False,
                'use_map': False,
                'use_external': False,
            },
            'results': merged_results,
        }

        split_output_dir = os.path.join(output_dir, eval_split)
        os.makedirs(split_output_dir, exist_ok=True)
        submission_path = os.path.join(split_output_dir, 'submission.json')
        with open(submission_path, 'w') as f:
            json.dump(submission, f)
        print(f'  Wrote submission ({len(merged_results)} samples) -> {submission_path}')

        # Run nuScenes tracking evaluation
        print(f'\n  Running nuScenes tracking evaluation for {eval_split} ...\n')
        cfg = config_factory('tracking_nips_2019')
        nusc_eval = TrackingEval(
            config=cfg,
            result_path=submission_path,
            eval_set=eval_split,
            output_dir=split_output_dir,
            nusc_version=args.version,
            nusc_dataroot=args.dataroot,
            verbose=True,
        )
        nusc_eval.main(render_curves=False)


if __name__ == '__main__':
    main()
