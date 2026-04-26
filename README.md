# Project_MOT

Multi-Object Tracking project using nuScenes data.

---

## nuScenes sanity check

Run the script below to verify that the nuScenes dataset is correctly mounted
and that all expected files are accessible:

```bash
python3 scripts/check_nuscenes.py \
    --dataroot /data/nuscenes \
    --version  v1.0-mini
```

### What the script checks

| Check | Description |
|---|---|
| `--dataroot` exists | The dataset root directory is mounted and readable |
| `--version` sub-directory exists | e.g. `v1.0-mini/` is present inside the root |
| Dataset summary | Prints scene / sample / sample_data / calibrated_sensor / ego_pose counts |
| First scene & sample | Name, token, timestamp, and available sensor channels |
| File existence | Verifies `LIDAR_TOP` and every `CAM_*` file path on disk |

### Arguments

| Argument | Default | Description |
|---|---|---|
| `--dataroot` | `/data/nuscenes` | Root directory of the nuScenes dataset |
| `--version` | `v1.0-mini` | Dataset version (`v1.0-mini`, `v1.0-trainval`, etc.) |

### Example output (healthy dataset)

```
Loading nuScenes v1.0-mini from /data/nuscenes …

=== Dataset summary ===
  Scenes            : 10
  Samples           : 404
  Sample data       : 31206
  Calibrated sensors: 148
  Ego poses         : 31206

=== First scene ===
  Name       : scene-0061
  Description: Parked truck, construction, intersection, turn
  Token      : cc8c0bf57f984915a77078b10eb33198

=== First sample ===
  Token    : ca9a282c9e77460f8360f564131a8af5
  Timestamp: 1532402927647951 µs
  Channels : CAM_BACK, CAM_BACK_LEFT, CAM_BACK_RIGHT, CAM_FRONT, CAM_FRONT_LEFT, CAM_FRONT_RIGHT, LIDAR_TOP, RADAR_BACK_LEFT, RADAR_BACK_RIGHT, RADAR_FRONT, RADAR_FRONT_LEFT, RADAR_FRONT_RIGHT

=== File existence check (first sample) ===
  [OK] LIDAR_TOP: /data/nuscenes/samples/LIDAR_TOP/...
  [OK] CAM_BACK : /data/nuscenes/samples/CAM_BACK/...
  ...

All file checks passed. Dataset looks healthy.
```
