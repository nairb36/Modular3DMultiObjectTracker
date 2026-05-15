#!/usr/bin/env python3
"""
Patch nuscenes-devkit and motmetrics for numpy >= 1.24 compatibility.

Run once after pip install:
    python3 scripts/patch_nuscenes_eval.py
"""

import importlib
import os
import sys


def find_file(module_name, relative_path):
    mod = importlib.import_module(module_name)
    base = os.path.dirname(mod.__file__)
    return os.path.join(base, relative_path)


def patch_file(path, replacements):
    with open(path) as f:
        content = f.read()

    changed = False
    for old, new in replacements:
        if old in content and new not in content:
            content = content.replace(old, new)
            changed = True

    if changed:
        with open(path, 'w') as f:
            f.write(content)
        print(f'  Patched {path}')
    else:
        print(f'  Already patched {path}')


def main():
    print('Patching nuscenes-devkit + motmetrics for numpy >= 1.24 ...\n')

    # 1. motmetrics/mot.py: np.bool -> np.bool_
    patch_file(
        find_file('motmetrics', 'mot.py'),
        [
            ('dtype=np.bool)', 'dtype=np.bool_)'),
        ]
    )

    # 2. nuscenes eval/tracking/mot.py: handle _indices as dict (motmetrics 1.2+ format)
    patch_file(
        find_file('nuscenes.eval.tracking', 'mot.py'),
        [
            (
                "        idx = pd.MultiIndex.from_tuples(indices, names=['FrameId', 'Event'])",
                "        if isinstance(indices, dict):\n"
                "            indices = list(zip(indices['FrameId'], indices['Event']))\n"
                "        idx = pd.MultiIndex.from_tuples(indices, names=['FrameId', 'Event'])"
            ),
        ]
    )

    # 3. nuscenes eval/tracking/utils.py: register pred_frequencies metric
    patch_file(
        find_file('nuscenes.eval.tracking', 'utils.py'),
        [
            (
                "        'num_frames', 'obj_frequencies', 'num_matches',",
                "        'num_frames', 'obj_frequencies', 'pred_frequencies', 'num_matches',",
            ),
        ]
    )

    # 4. nuscenes eval/tracking/metrics.py: add **kwargs to custom metric functions
    patch_file(
        find_file('nuscenes.eval.tracking', 'metrics.py'),
        [
            ('def motar(df: DataFrame, num_matches: int, num_misses: int, num_switches: int, num_false_positives: int,\n          num_objects: int, alpha: float = 1.0)',
             'def motar(df: DataFrame, num_matches: int, num_misses: int, num_switches: int, num_false_positives: int,\n          num_objects: int, alpha: float = 1.0, **kwargs)'),
            ('def mota_custom(df: DataFrame, num_misses: int, num_switches: int, num_false_positives: int, num_objects: int)',
             'def mota_custom(df: DataFrame, num_misses: int, num_switches: int, num_false_positives: int, num_objects: int, **kwargs)'),
            ('def motp_custom(df: DataFrame, num_detections: float)',
             'def motp_custom(df: DataFrame, num_detections: float, **kwargs)'),
            ('def faf(df: DataFrame, num_false_positives: float, num_frames: float)',
             'def faf(df: DataFrame, num_false_positives: float, num_frames: float, **kwargs)'),
            ('def num_fragmentations_custom(df: DataFrame, obj_frequencies: DataFrame)',
             'def num_fragmentations_custom(df: DataFrame, obj_frequencies: DataFrame, **kwargs)'),
            ('def track_initialization_duration(df: DataFrame, obj_frequencies: DataFrame)',
             'def track_initialization_duration(df: DataFrame, obj_frequencies: DataFrame, **kwargs)'),
            ('def longest_gap_duration(df: DataFrame, obj_frequencies: DataFrame)',
             'def longest_gap_duration(df: DataFrame, obj_frequencies: DataFrame, **kwargs)'),
        ]
    )

    print('\nDone. You can now run: python3 scripts/evaluate.py <result_files>')


if __name__ == '__main__':
    main()
