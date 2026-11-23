# RamanSeg

RamanSeg contains scripts and experiment outputs for RGB point-cloud processing, robot-camera hand-eye calibration, multi-frame fusion, and unsupervised color analysis.

## Main workflows

- `scripts/HandEyeCalibration_class_fixed.py` and `scripts/prepare_gemini_robot_poses.py`: Gemini hand-eye calibration using the recorded robot TCP poses and camera intrinsics.
- `scripts/transform_and_merge_pointclouds.py`: transforms colored point clouds into robot-base coordinates and merges frames.
- `scripts/unsupervised_stage1/`: RGB-only point-cloud clustering and visualization, including K=3 and agarose K=6/K=7 runs.
- `scripts/stage3/run_grid4x6_classification.py`: summarizes existing point-level K=6 labels by a 4-column × 6-row XY grid.
- `scripts/stage3b/run_grid_sample_rgb_clustering.py`: extracts RGB statistics for each grid cell and clusters the 24 grid samples with K=6.

## Current experiment outputs

- Hand-eye calibration: `calibration/handeye_gemini_1110/`, `1119/`, and `1120/`; reports are under `reports/handeye_gemini_*`.
- Point-level agarose RGB clustering: `reports/agarose_rgb_k6/`.
- 4×6 point-label grid summary: `reports/stage3_grid4x6/`.
- 24-sample RGB-statistics clustering: `reports/stage3b_grid_sample_rgb/`.
- Other RGB overlay and tissue experiments are stored in their corresponding `reports/` subdirectories.

## Data and execution

Raw and processed datasets live under `data/` and are intentionally excluded from Git. Place the required RamanSeg PLY/CSV inputs at the paths configured in the scripts, then run the desired Python script from the repository root. Generated reports and figures are written under `reports/`.

The point-cloud clustering workflows are unsupervised. Cluster IDs are numeric group labels and do not automatically represent concentrations or tissue semantics.
