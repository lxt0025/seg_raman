#!/usr/bin/env python3

"""Rename the original 1110 outputs to the dated multi-dataset layout."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CALIBRATION_OLD = ROOT / "calibration/handeye_gemini"
CALIBRATION_NEW = ROOT / "calibration/handeye_gemini_1110"
REPORTS_OLD = ROOT / "reports/handeye_gemini"
REPORTS_NEW = ROOT / "reports/handeye_gemini_1110"
PROCESSED_NEW = ROOT / "data/processed/1110"
PER_FRAME_OLD = ROOT / "data/processed/per_frame_base"
PER_FRAME_NEW = PROCESSED_NEW / "per_frame_base"
MERGED_DIR_OLD = ROOT / "data/processed/merged"
MERGED_DIR_NEW = PROCESSED_NEW / "merged"
MERGED_NAME_OLD = "ramanseg_gemini_merged_robot_base.ply"
MERGED_NAME_NEW = "ramanseg_1110_gemini_merged_robot_base.ply"


def require_source_and_free_target(source, target):
    if not source.exists():
        raise FileNotFoundError(source)
    if target.exists():
        raise FileExistsError(target)


def main():
    moves = [
        (CALIBRATION_OLD, CALIBRATION_NEW),
        (REPORTS_OLD, REPORTS_NEW),
        (PER_FRAME_OLD, PER_FRAME_NEW),
        (MERGED_DIR_OLD, MERGED_DIR_NEW),
    ]
    for source, target in moves:
        require_source_and_free_target(source, target)

    PROCESSED_NEW.mkdir(parents=True, exist_ok=False)
    CALIBRATION_OLD.rename(CALIBRATION_NEW)
    REPORTS_OLD.rename(REPORTS_NEW)
    PER_FRAME_OLD.rename(PER_FRAME_NEW)
    MERGED_DIR_OLD.rename(MERGED_DIR_NEW)

    merged_old = MERGED_DIR_NEW / MERGED_NAME_OLD
    merged_new = MERGED_DIR_NEW / MERGED_NAME_NEW
    require_source_and_free_target(merged_old, merged_new)
    merged_old.rename(merged_new)

    pose_json = REPORTS_NEW / "pose_validation.json"
    pose = json.loads(pose_json.read_text(encoding="utf-8"))
    pose["raw_input_directory"] = str((ROOT / "data/raw/1110").resolve())
    pose["pose_output_directory"] = str(
        (CALIBRATION_NEW / "robot_poses").resolve()
    )
    pose_json.write_text(
        json.dumps(pose, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    fusion_json = REPORTS_NEW / "fusion_stats.json"
    fusion = json.loads(fusion_json.read_text(encoding="utf-8"))
    fusion["transformed_directory"] = str(PER_FRAME_NEW.resolve())
    fusion["handeye_path"] = str(
        (CALIBRATION_NEW / "FinalTransforms/T_cam2gripper_selected.npz").resolve()
    )
    fusion["pose_directory"] = str(
        (CALIBRATION_NEW / "robot_poses").resolve()
    )
    fusion["cloud_directory"] = str(
        (ROOT / "data/raw/1110/pointcloud").resolve()
    )
    fusion["merged_output"] = str(merged_new.resolve())
    fusion["source_frame_count"] = int(fusion["processed_frame_count"])
    fusion_json.write_text(
        json.dumps(fusion, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    replacements = {
        str(CALIBRATION_OLD): str(CALIBRATION_NEW),
        str(REPORTS_OLD): str(REPORTS_NEW),
        str(PER_FRAME_OLD): str(PER_FRAME_NEW),
        str(ROOT / "data/processed/merged" / MERGED_NAME_OLD): str(merged_new),
        str(ROOT / "data/raw/pointcloud"): str(ROOT / "data/raw/1110/pointcloud"),
        "calibration/handeye_gemini/": "calibration/handeye_gemini_1110/",
        "reports/handeye_gemini/": "reports/handeye_gemini_1110/",
        "data/processed/per_frame_base": "data/processed/1110/per_frame_base",
        "data/processed/merged/" + MERGED_NAME_OLD: (
            "data/processed/1110/merged/" + MERGED_NAME_NEW
        ),
        "data/raw/pointcloud": "data/raw/1110/pointcloud",
    }
    for path in REPORTS_NEW.iterdir():
        if not path.is_file() or path.suffix not in {".md", ".log", ".json"}:
            continue
        text = path.read_text(encoding="utf-8")
        for old, new in replacements.items():
            text = text.replace(old, new)
        path.write_text(text, encoding="utf-8")

    print("Calibration:", CALIBRATION_NEW)
    print("Reports:", REPORTS_NEW)
    print("Per-frame clouds:", PER_FRAME_NEW)
    print("Merged cloud:", merged_new)


if __name__ == "__main__":
    main()
