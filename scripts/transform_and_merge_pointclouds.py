#!/usr/bin/env python3

"""Transform per-frame RGB-D point clouds into the robot-base frame.

Transformation chain used for every included frame i:

    T_cam2base_i = T_gripper2base_i @ T_cam2gripper
    p_base       = T_cam2base_i @ p_cam

Assumptions:
1. Each input PLY is expressed in the RGB camera coordinate system.
2. The input PLY coordinates use metres by default.
3. Every robot-pose .npy is a 4x4 gripper-to-base transform in metres.
4. The hand-eye .npz contains a 4x4 camera-to-gripper matrix under key "T".
5. Point-cloud and robot-pose frame numbers both start at frame-000001.
"""

import argparse
import glob
import json
import os
import re
import sys

import numpy as np
import open3d as o3d


REPOSITORY_ROOT = os.path.dirname(
    os.path.dirname(os.path.abspath(__file__))
)

# Defaults adapted to RamanSeg while preserving the original algorithm.
DEFAULT_CLOUD_DIR = os.path.join(
    REPOSITORY_ROOT, "data", "raw", "pointcloud"
)
DEFAULT_HANDEYE_PATH = os.path.join(
    REPOSITORY_ROOT,
    "calibration", "handeye_gemini", "FinalTransforms",
    "T_cam2gripper_selected.npz",
)
DEFAULT_POSE_DIR = os.path.join(
    REPOSITORY_ROOT, "calibration", "handeye_gemini", "robot_poses"
)
DEFAULT_OUTPUT_PATH = os.path.join(
    REPOSITORY_ROOT,
    "data", "processed", "merged",
    "ramanseg_gemini_merged_robot_base.ply",
)
DEFAULT_TRANSFORMED_DIR = os.path.join(
    REPOSITORY_ROOT, "data", "processed", "per_frame_base"
)
DEFAULT_STATS_JSON = os.path.join(
    REPOSITORY_ROOT, "reports", "handeye_gemini", "fusion_stats.json"
)

DEFAULT_CLOUD_PATTERN = (
    "frame-*.pointcloud_color_rgb_correct.ply"
)

DEFAULT_EXCLUDED_POSE_FRAME_IDS = []


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Transform aligned RGB-D point clouds from the RGB-camera "
            "frame to the robot-base frame and merge them."
        )
    )

    parser.add_argument(
        "--cloud_dir",
        default=DEFAULT_CLOUD_DIR,
        help="Directory containing the per-frame PLY files.",
    )

    parser.add_argument(
        "--cloud_pattern",
        default=DEFAULT_CLOUD_PATTERN,
        help="Glob pattern used inside cloud_dir.",
    )

    parser.add_argument(
        "--handeye",
        default=DEFAULT_HANDEYE_PATH,
        help="Path to T_cam2gripper_PARK.npz.",
    )

    parser.add_argument(
        "--pose_dir",
        default=DEFAULT_POSE_DIR,
        help="Directory containing frame-XXXXXX.npy gripper2base poses.",
    )

    parser.add_argument(
        "--output",
        default=DEFAULT_OUTPUT_PATH,
        help="Output path of the merged robot-base point cloud.",
    )

    parser.add_argument(
        "--transformed_dir",
        default=DEFAULT_TRANSFORMED_DIR,
        help="Directory used when --save_transformed_frames is enabled.",
    )

    parser.add_argument(
        "--exclude_frames",
        type=int,
        nargs="*",
        default=DEFAULT_EXCLUDED_POSE_FRAME_IDS,
        help=(
            "Robot-pose frame IDs excluded from point-cloud fusion. "
            "Default: none."
        ),
    )

    parser.add_argument(
        "--frame_offset",
        type=int,
        default=0,
        help=(
            "pose_frame_id = cloud_frame_id + frame_offset. "
            "Both datasets currently start at 1, so the default is 0."
        ),
    )

    parser.add_argument(
        "--cloud_unit",
        choices=["m", "mm"],
        default="m",
        help="Coordinate unit used by the input PLY files.",
    )

    parser.add_argument(
        "--voxel_size",
        type=float,
        default=0.002,
        help=(
            "Voxel size in metres applied before and during merging. "
            "Use 0 to disable downsampling. Default: 0.002 m."
        ),
    )

    parser.add_argument(
        "--merge_cleanup_interval",
        type=int,
        default=5,
        help=(
            "Downsample the accumulated cloud after this many processed "
            "frames to control memory use. Default: 5."
        ),
    )

    parser.add_argument(
        "--save_transformed_frames",
        action="store_true",
        help="Also save every transformed per-frame point cloud.",
    )

    parser.add_argument(
        "--stats_json",
        default=DEFAULT_STATS_JSON,
        help="Write machine-readable fusion statistics to this JSON file.",
    )

    parser.add_argument(
        "--expected_frame_count",
        type=int,
        default=None,
        help=(
            "Require exactly this many contiguous frames starting at 1. "
            "When omitted, the count is inferred from the input files."
        ),
    )

    return parser.parse_args()


def extract_frame_id(path):
    """Extract the integer ID from a name containing frame-XXXXXX."""
    match = re.search(
        r"frame-(\d+)",
        os.path.basename(path),
        flags=re.IGNORECASE,
    )

    if match is None:
        raise ValueError(
            "Cannot extract frame ID from: {}".format(path)
        )

    return int(match.group(1))


def validate_transform(T, label):
    """Validate a 4x4 rigid homogeneous transformation matrix."""
    T = np.asarray(T, dtype=np.float64)

    if T.shape != (4, 4):
        raise ValueError(
            "{} must be 4x4, but shape is {}".format(
                label,
                T.shape,
            )
        )

    if not np.all(np.isfinite(T)):
        raise ValueError(
            "{} contains NaN or Inf".format(label)
        )

    if not np.allclose(
        T[3, :],
        np.array([0.0, 0.0, 0.0, 1.0]),
        atol=1e-6,
    ):
        raise ValueError(
            "{} has an invalid homogeneous last row: {}".format(
                label,
                T[3, :],
            )
        )

    R = T[:3, :3]

    if not np.allclose(
        R.T.dot(R),
        np.eye(3),
        atol=1e-3,
    ):
        raise ValueError(
            "{} has a non-orthonormal rotation".format(label)
        )

    determinant = np.linalg.det(R)
    if not np.isclose(
        determinant,
        1.0,
        atol=1e-3,
    ):
        raise ValueError(
            "{} rotation determinant is {} instead of 1".format(
                label,
                determinant,
            )
        )

    return T


def load_handeye_transform(path):
    """Load camera-to-gripper transform from the saved PARK NPZ file."""
    if not os.path.isfile(path):
        raise FileNotFoundError(
            "Hand-eye file not found: {}".format(path)
        )

    data = np.load(path)

    if "T" in data.files:
        T = data["T"]
    elif "arr_0" in data.files:
        # Compatibility with older unnamed np.savez output.
        T = data["arr_0"]
    else:
        raise KeyError(
            "No 'T' or 'arr_0' in {}. Available keys: {}".format(
                path,
                data.files,
            )
        )

    T = validate_transform(
        T,
        "T_cam2gripper",
    )

    print("Hand-eye file:", path)
    print("NPZ fields:", data.files)
    print("T_cam2gripper:\n", T)
    print(
        "T_cam2gripper translation/mm:",
        T[:3, 3] * 1000.0,
    )

    if "fixed_target_translation_residual_mm" in data.files:
        print(
            "Hand-eye fixed-target translation residual/mm:",
            float(data["fixed_target_translation_residual_mm"]),
        )

    if "fixed_target_rotation_residual_deg" in data.files:
        print(
            "Hand-eye fixed-target rotation residual/deg:",
            float(data["fixed_target_rotation_residual_deg"]),
        )

    if "valid_frame_ids" in data.files:
        print(
            "Hand-eye calibration valid frame IDs:",
            data["valid_frame_ids"],
        )

    return T


def load_robot_pose(path):
    """Load one 4x4 gripper-to-base transform from an NPY file."""
    if not os.path.isfile(path):
        raise FileNotFoundError(
            "Robot pose not found: {}".format(path)
        )

    T = np.asarray(
        np.load(path),
        dtype=np.float64,
    )

    return validate_transform(
        T,
        "T_gripper2base ({})".format(path),
    )


def clean_and_scale_point_cloud(point_cloud, cloud_unit):
    """Remove invalid vertices and convert millimetres to metres if needed."""
    points = np.asarray(point_cloud.points)

    if points.size == 0:
        return o3d.geometry.PointCloud()

    valid = np.all(np.isfinite(points), axis=1)

    if not np.all(valid):
        cleaned = o3d.geometry.PointCloud()

        cleaned_points = points[valid].copy()
        cleaned.points = o3d.utility.Vector3dVector(
            cleaned_points
        )

        colors = np.asarray(point_cloud.colors)
        if len(colors) == len(points):
            cleaned.colors = o3d.utility.Vector3dVector(
                colors[valid].copy()
            )

        normals = np.asarray(point_cloud.normals)
        if len(normals) == len(points):
            cleaned.normals = o3d.utility.Vector3dVector(
                normals[valid].copy()
            )

        point_cloud = cleaned
        points = np.asarray(point_cloud.points)

    if cloud_unit == "mm":
        points = points.copy() / 1000.0
        point_cloud.points = o3d.utility.Vector3dVector(points)

    return point_cloud


def print_cloud_bounds(label, point_cloud):
    points = np.asarray(point_cloud.points)

    if len(points) == 0:
        print(label, "is empty")
        return

    minimum = np.min(points, axis=0)
    maximum = np.max(points, axis=0)

    print(label, "min/m:", minimum)
    print(label, "max/m:", maximum)
    print(label, "range/m:", maximum - minimum)


def main():
    args = parse_arguments()

    if args.voxel_size < 0:
        raise ValueError("voxel_size must be non-negative")

    if args.merge_cleanup_interval < 1:
        raise ValueError(
            "merge_cleanup_interval must be at least 1"
        )

    cloud_search = os.path.join(
        args.cloud_dir,
        args.cloud_pattern,
    )

    cloud_files = sorted(
        glob.glob(cloud_search),
        key=extract_frame_id,
    )

    if not cloud_files:
        raise FileNotFoundError(
            "No point clouds matched: {}".format(cloud_search)
        )

    # Detect duplicate point-cloud frame IDs before processing.
    cloud_frame_ids = [
        extract_frame_id(path)
        for path in cloud_files
    ]

    if len(cloud_frame_ids) != len(set(cloud_frame_ids)):
        raise ValueError(
            "Duplicate point-cloud frame IDs detected: {}".format(
                cloud_frame_ids
            )
        )

    expected_count = (
        args.expected_frame_count
        if args.expected_frame_count is not None
        else len(cloud_frame_ids)
    )
    expected_frame_ids = list(range(1, expected_count + 1))
    if cloud_frame_ids != expected_frame_ids:
        raise ValueError(
            "RamanSeg fusion requires point-cloud frame IDs 1..{}; got {}".format(
                expected_count, cloud_frame_ids
            )
        )

    excluded_pose_frame_ids = set(args.exclude_frames)

    print("Point-cloud directory:", args.cloud_dir)
    print("Point-cloud pattern:", args.cloud_pattern)
    print("Point clouds found:", len(cloud_files))
    print("Point-cloud frame IDs:", cloud_frame_ids)
    print("Robot-pose directory:", args.pose_dir)
    print("Frame offset:", args.frame_offset)
    print(
        "Excluded robot-pose frame IDs:",
        sorted(excluded_pose_frame_ids),
    )
    print("Input cloud unit:", args.cloud_unit)
    print("Voxel size/m:", args.voxel_size)

    T_cam2gripper = load_handeye_transform(
        args.handeye
    )

    if args.save_transformed_frames:
        os.makedirs(
            args.transformed_dir,
            exist_ok=True,
        )

    output_parent = os.path.dirname(
        os.path.abspath(args.output)
    )
    os.makedirs(output_parent, exist_ok=True)

    merged = o3d.geometry.PointCloud()
    processed_pose_frame_ids = []
    skipped_pose_frame_ids = []

    total_input_points = 0
    total_points_after_frame_downsampling = 0

    for cloud_index, cloud_path in enumerate(
        cloud_files,
        start=1,
    ):
        cloud_frame_id = extract_frame_id(
            cloud_path
        )

        pose_frame_id = (
            cloud_frame_id
            + args.frame_offset
        )

        # Exclude frames according to the robot-pose/RGB frame ID. RamanSeg's
        # default exclusion list is empty, including calibration-failed frame 42.
        if pose_frame_id in excluded_pose_frame_ids:
            print(
                "[{}/{}] SKIP excluded frame: cloud={:06d}, "
                "pose={:06d}".format(
                    cloud_index,
                    len(cloud_files),
                    cloud_frame_id,
                    pose_frame_id,
                )
            )
            skipped_pose_frame_ids.append(pose_frame_id)
            continue

        pose_path = os.path.join(
            args.pose_dir,
            "frame-{:06d}.npy".format(pose_frame_id),
        )

        T_gripper2base = load_robot_pose(
            pose_path
        )

        # This is the camera-to-world transform when robot base is world.
        T_cam2base = (
            T_gripper2base
            .dot(T_cam2gripper)
        )

        point_cloud = o3d.io.read_point_cloud(
            cloud_path
        )

        point_cloud = clean_and_scale_point_cloud(
            point_cloud,
            args.cloud_unit,
        )

        input_count = len(point_cloud.points)
        if input_count == 0:
            raise ValueError(
                "Point cloud is empty after loading/cleaning: {}".format(
                    cloud_path
                )
            )

        total_input_points += input_count

        # Transform local RGB-camera points into the common robot-base frame.
        point_cloud.transform(T_cam2base)

        if args.voxel_size > 0:
            point_cloud = point_cloud.voxel_down_sample(
                voxel_size=args.voxel_size
            )

        frame_output_count = len(point_cloud.points)
        total_points_after_frame_downsampling += frame_output_count

        if args.save_transformed_frames:
            transformed_path = os.path.join(
                args.transformed_dir,
                (
                    "frame-{:06d}.pointcloud_"
                    "in_robot_base.ply"
                ).format(pose_frame_id),
            )

            success = o3d.io.write_point_cloud(
                transformed_path,
                point_cloud,
                write_ascii=False,
                compressed=False,
            )

            if not success:
                raise IOError(
                    "Failed to save transformed cloud: {}".format(
                        transformed_path
                    )
                )

        merged += point_cloud
        processed_pose_frame_ids.append(pose_frame_id)

        # Periodically downsample the accumulated cloud to prevent the full
        # multi-frame point set from exhausting system memory.
        if (
            args.voxel_size > 0
            and len(processed_pose_frame_ids)
            % args.merge_cleanup_interval == 0
        ):
            merged = merged.voxel_down_sample(
                voxel_size=args.voxel_size
            )

        print(
            "[{}/{}] cloud={:06d} -> pose={:06d}, "
            "input_points={}, transformed_points={}, "
            "merged_points={}".format(
                cloud_index,
                len(cloud_files),
                cloud_frame_id,
                pose_frame_id,
                input_count,
                frame_output_count,
                len(merged.points),
            )
        )

    if not processed_pose_frame_ids:
        raise RuntimeError(
            "No point clouds were processed"
        )

    if args.voxel_size > 0:
        merged = merged.voxel_down_sample(
            voxel_size=args.voxel_size
        )

    print("\nFusion summary")
    print("Processed pose frame IDs:", processed_pose_frame_ids)
    print("Skipped pose frame IDs:", skipped_pose_frame_ids)
    print("Processed frame count:", len(processed_pose_frame_ids))
    print("Total input points:", total_input_points)
    print(
        "Total per-frame points after downsampling:",
        total_points_after_frame_downsampling,
    )
    print("Final merged points:", len(merged.points))
    print_cloud_bounds("Final merged cloud", merged)

    merged_points = np.asarray(merged.points)
    bounds_min = np.min(merged_points, axis=0)
    bounds_max = np.max(merged_points, axis=0)

    success = o3d.io.write_point_cloud(
        args.output,
        merged,
        write_ascii=False,
        compressed=False,
    )

    if not success:
        raise IOError(
            "Failed to save merged point cloud: {}".format(
                args.output
            )
        )

    print("Saved merged point cloud:", args.output)

    expected_processed_count = expected_count - len(excluded_pose_frame_ids)
    if len(processed_pose_frame_ids) != expected_processed_count:
        raise RuntimeError(
            "Expected {} fusion frames, processed {}".format(
                expected_processed_count, len(processed_pose_frame_ids)
            )
        )
    if set(skipped_pose_frame_ids) != excluded_pose_frame_ids:
        raise RuntimeError(
            "Skipped frames do not equal requested exclusions: {} vs {}".format(
                skipped_pose_frame_ids, sorted(excluded_pose_frame_ids)
            )
        )

    stats_parent = os.path.dirname(os.path.abspath(args.stats_json))
    os.makedirs(stats_parent, exist_ok=True)
    stats = {
        "processed_frame_ids": processed_pose_frame_ids,
        "skipped_frame_ids": skipped_pose_frame_ids,
        "processed_frame_count": len(processed_pose_frame_ids),
        "source_frame_count": expected_count,
        "total_input_points": int(total_input_points),
        "total_per_frame_points_after_downsampling": int(
            total_points_after_frame_downsampling
        ),
        "final_merged_points": int(len(merged.points)),
        "xyz_min_m": bounds_min.tolist(),
        "xyz_max_m": bounds_max.tolist(),
        "xyz_range_m": (bounds_max - bounds_min).tolist(),
        "cloud_unit": args.cloud_unit,
        "voxel_size_m": args.voxel_size,
        "merge_cleanup_interval": args.merge_cleanup_interval,
        "frame_offset": args.frame_offset,
        "excluded_frame_ids": sorted(excluded_pose_frame_ids),
        "save_transformed_frames": bool(args.save_transformed_frames),
        "transformed_directory": os.path.abspath(args.transformed_dir),
        "handeye_path": os.path.abspath(args.handeye),
        "pose_directory": os.path.abspath(args.pose_dir),
        "cloud_directory": os.path.abspath(args.cloud_dir),
        "merged_output": os.path.abspath(args.output),
    }
    with open(args.stats_json, "w", encoding="utf-8") as stream:
        json.dump(stats, stream, ensure_ascii=False, indent=2)
    print("Saved fusion statistics:", args.stats_json)


if __name__ == "__main__":
    try:
        main()
    except Exception as error:
        print("ERROR:", error, file=sys.stderr)
        raise
