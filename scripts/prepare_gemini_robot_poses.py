#!/usr/bin/env python3

"""Validate RamanSeg inputs and create JAKA TCP gripper-to-base poses."""

import json
import re
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT / "data/raw"
POSE_DIR = ROOT / "calibration/handeye_gemini/robot_poses"
REPORT_DIR = ROOT / "reports/handeye_gemini"
EXPECTED_IDS = list(range(1, 76))

# JAKA SDK rpy_to_rot_matrix reference example. The RotMatrix C struct stores
# rows as x.*, y.*, z.*; the Python SDK documents the return as row-major.
# This is exactly Rz(rz) @ Ry(ry) @ Rx(rx), i.e. scipy's extrinsic "xyz".
JAKA_REFERENCE_RPY = np.array(
    [-1.81826, -0.834253, -2.30243], dtype=np.float64
)
JAKA_REFERENCE_ROTATION = np.array(
    [
        [-0.448774, -0.662098, 0.600190],
        [-0.499824, -0.370777, -0.782751],
        [0.740795, -0.651268, -0.164538],
    ],
    dtype=np.float64,
)


def frame_id(path):
    match = re.search(r"frame-(\d+)", path.name)
    if match is None:
        raise ValueError("Cannot extract frame ID from {}".format(path))
    return int(match.group(1))


def indexed_files(folder, pattern):
    result = {}
    for path in folder.glob(pattern):
        identifier = frame_id(path)
        if identifier in result:
            raise ValueError(
                "Duplicate frame {}: {} and {}".format(
                    identifier, result[identifier], path
                )
            )
        result[identifier] = path
    if sorted(result) != EXPECTED_IDS:
        raise ValueError(
            "{} / {} IDs do not equal 1..75; got {}".format(
                folder, pattern, sorted(result)
            )
        )
    return result


def read_ply_vertex_count(path):
    with path.open("rb") as stream:
        first = stream.readline().decode("ascii", errors="strict").strip()
        if first != "ply":
            raise ValueError("Not a PLY file: {}".format(path))
        count = None
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PLY header has no end_header: {}".format(path))
            text = line.decode("ascii", errors="strict").strip()
            if text.startswith("element vertex "):
                count = int(text.split()[-1])
            if text == "end_header":
                break
    if count is None or count <= 0:
        raise ValueError("PLY has no positive vertex count: {}".format(path))
    return count


def jaka_rpy_to_rotation_matrix(rpy):
    """Numerical convention of JAKA SDK rpy_to_rot_matrix([rx, ry, rz])."""
    rpy = np.asarray(rpy, dtype=np.float64)
    if rpy.shape != (3,) or not np.all(np.isfinite(rpy)):
        raise ValueError("RPY must contain three finite radians")
    return Rotation.from_euler("xyz", rpy, degrees=False).as_matrix()


def validate_sdk_convention():
    actual = jaka_rpy_to_rotation_matrix(JAKA_REFERENCE_RPY)
    error = float(np.max(np.abs(actual - JAKA_REFERENCE_ROTATION)))
    if error > 1e-6:
        raise RuntimeError(
            "JAKA rpy_to_rot_matrix reference check failed: max error {}".format(
                error
            )
        )
    return error


def validate_rotation(rotation, label):
    orthogonality_error = float(
        np.max(np.abs(rotation.T @ rotation - np.eye(3)))
    )
    determinant = float(np.linalg.det(rotation))
    if orthogonality_error > 1e-10:
        raise ValueError("{} rotation is not orthonormal".format(label))
    if not np.isclose(determinant, 1.0, atol=1e-10):
        raise ValueError(
            "{} rotation determinant is {}".format(label, determinant)
        )
    return orthogonality_error, determinant


def main():
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    POSE_DIR.mkdir(parents=True, exist_ok=True)

    sdk_reference_error = validate_sdk_convention()

    images = indexed_files(RAW / "images", "frame-*.color.png")
    frame_json = indexed_files(RAW / "frame_info", "frame-*.info.json")
    clouds = indexed_files(
        RAW / "pointcloud", "frame-*.pointcloud_color_rgb_correct.ply"
    )
    depth_png = indexed_files(
        RAW / "depth", "frame-*.aligned_depth_raw_uint16.png"
    )
    depth_npy = indexed_files(
        RAW / "depth", "frame-*.aligned_depth_m.npy"
    )

    with (RAW / "robot_session_info.json").open(
        "r", encoding="utf-8"
    ) as stream:
        session = json.load(stream)
    if session["session_locked_tool_id"] != 1:
        raise ValueError("Robot session is not locked to Tool ID 1")
    if session["startup_user_frame_id"] != 0:
        raise ValueError("Robot session did not start in User Frame ID 0")
    if session["tool_offset_verified_by_sdk"] is not False:
        raise ValueError("Unexpected tool-offset verification state")

    translations = []
    determinants = []
    orthogonality_errors = []
    input_point_count = 0
    per_frame = []

    for identifier in EXPECTED_IDS:
        image = cv2.imread(str(images[identifier]), cv2.IMREAD_COLOR)
        if image is None or image.shape[:2] != (720, 1280):
            raise ValueError(
                "Frame {} RGB is unreadable or not 1280x720".format(identifier)
            )
        depth = cv2.imread(
            str(depth_png[identifier]), cv2.IMREAD_UNCHANGED
        )
        if depth is None or depth.shape != (720, 1280):
            raise ValueError(
                "Frame {} aligned depth PNG is invalid".format(identifier)
            )
        depth_array = np.load(depth_npy[identifier], mmap_mode="r")
        if depth_array.shape != (720, 1280):
            raise ValueError(
                "Frame {} aligned depth NPY is invalid".format(identifier)
            )

        vertices = read_ply_vertex_count(clouds[identifier])
        input_point_count += vertices

        with frame_json[identifier].open("r", encoding="utf-8") as stream:
            info = json.load(stream)
        if info["frame_index"] != identifier:
            raise ValueError("Frame index mismatch in {}".format(frame_json[identifier]))
        robot = info["robot"]
        if robot["tool_id"] != 1 or robot["session_locked_tool_id"] != 1:
            raise ValueError("Frame {} does not use Tool ID 1".format(identifier))
        if robot["user_frame_id"] != 0:
            raise ValueError("Frame {} does not use User Frame ID 0".format(identifier))
        if robot["pose_api"] != "get_actual_tcp_position":
            raise ValueError("Unexpected robot pose API in frame {}".format(identifier))
        if robot["translation_unit"] != "millimeter":
            raise ValueError("Unexpected translation unit in frame {}".format(identifier))
        if robot["rotation_unit"] != "radian":
            raise ValueError("Unexpected rotation unit in frame {}".format(identifier))

        pose = np.asarray(robot["tcp_pose_raw"], dtype=np.float64)
        if pose.shape != (6,) or not np.all(np.isfinite(pose)):
            raise ValueError("Invalid tcp_pose_raw in frame {}".format(identifier))
        rotation = jaka_rpy_to_rotation_matrix(pose[3:])
        orthogonality_error, determinant = validate_rotation(
            rotation, "frame-{:06d}".format(identifier)
        )

        transform = np.eye(4, dtype=np.float64)
        transform[:3, :3] = rotation
        transform[:3, 3] = pose[:3] / 1000.0
        np.save(POSE_DIR / "frame-{:06d}.npy".format(identifier), transform)

        translations.append(transform[:3, 3])
        determinants.append(determinant)
        orthogonality_errors.append(orthogonality_error)
        per_frame.append(
            {
                "frame_id": identifier,
                "tcp_pose_raw_mm_rad": pose.tolist(),
                "translation_m": transform[:3, 3].tolist(),
                "det_R": determinant,
                "max_abs_RtR_minus_I": orthogonality_error,
                "pointcloud_vertices": vertices,
            }
        )

    saved = indexed_files(POSE_DIR, "frame-*.npy")
    for identifier, path in saved.items():
        transform = np.load(path)
        if transform.shape != (4, 4):
            raise ValueError("Saved pose is not 4x4: {}".format(path))
        if not np.array_equal(transform[3], [0.0, 0.0, 0.0, 1.0]):
            raise ValueError("Saved pose has invalid last row: {}".format(path))
        validate_rotation(transform[:3, :3], str(path))

    translations = np.asarray(translations)
    report = {
        "status": "passed",
        "frame_ids": EXPECTED_IDS,
        "frame_count": 75,
        "rgb_count": len(images),
        "depth_png_count": len(depth_png),
        "depth_npy_count": len(depth_npy),
        "frame_json_count": len(frame_json),
        "pointcloud_count": len(clouds),
        "pose_count": len(saved),
        "tool_id": 1,
        "user_frame_id": 0,
        "pose_direction": "T_gripper2base (active TCP to robot base)",
        "translation_input_unit": "mm",
        "translation_output_unit": "m",
        "rpy_input_unit": "rad",
        "rpy_conversion": "JAKA SDK rpy_to_rot_matrix; Rz(rz) @ Ry(ry) @ Rx(rx)",
        "jaka_reference_max_abs_error": sdk_reference_error,
        "max_rotation_orthogonality_error": max(orthogonality_errors),
        "rotation_determinant_min": min(determinants),
        "rotation_determinant_max": max(determinants),
        "translation_min_m": translations.min(axis=0).tolist(),
        "translation_max_m": translations.max(axis=0).tolist(),
        "pointcloud_header_vertex_total": input_point_count,
        "pose_output_directory": str(POSE_DIR.resolve()),
        "per_frame": per_frame,
    }
    report_path = REPORT_DIR / "pose_validation.json"
    with report_path.open("w", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)

    print("Validated 75 RGB, depth, frame JSON, and point-cloud inputs.")
    print("Generated 75 T_gripper2base poses:", POSE_DIR)
    print("Max R.T @ R error:", max(orthogonality_errors))
    print("det(R) range:", min(determinants), max(determinants))
    print("Pose validation report:", report_path)


if __name__ == "__main__":
    main()
