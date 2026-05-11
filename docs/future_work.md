# Future Work

## Detection
- CenterPoint detector integration
- PointPillars detector integration

## Motion Model
- CTRV motion model with EKF/UKF
- Constant acceleration model

## Cost Function
- Rotated 3D IoU (yaw-aware)
- Mahalanobis distance
- Deep embedding / appearance matching (DeepSORT-style)

## Association
- LAPJV solver (faster than Hungarian for large matrices)
- Cascaded matching (prioritize recent tracks)

## General
- Config file parser (YAML/JSON) for all parameters
- Output results to file for evaluation
- AMOTA/MOTA evaluation metrics
- Visualization / BEV rendering
