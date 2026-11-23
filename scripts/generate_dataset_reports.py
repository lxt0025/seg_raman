#!/usr/bin/env python3

"""Generate calibration and fusion Markdown reports for one capture folder."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np


def matrix_markdown(matrix):
    return "```text\n" + "\n".join(
        "[" + ", ".join("{:.12f}".format(v) for v in row) + "]"
        for row in matrix
    ) + "\n```"


def path_list(paths):
    return "\n".join("- `{}`".format(p.resolve()) for p in paths)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--calibration-dir", type=Path, required=True)
    parser.add_argument("--report-dir", type=Path, required=True)
    parser.add_argument("--processed-dir", type=Path, required=True)
    args = parser.parse_args()

    calibration = args.calibration_dir.resolve()
    final = calibration / "FinalTransforms"
    reports = args.report_dir.resolve()
    processed = args.processed_dir.resolve()
    reports.mkdir(parents=True, exist_ok=True)

    selected_path = final / "T_cam2gripper_selected.npz"
    pose_path = reports / "pose_validation.json"
    fusion_path = reports / "fusion_stats.json"
    selected = np.load(selected_path)
    pose = json.loads(pose_path.read_text(encoding="utf-8"))
    fusion = json.loads(fusion_path.read_text(encoding="utf-8"))

    T = np.asarray(selected["T"], dtype=np.float64)
    method = str(selected["method"])
    valid_ids = selected["valid_frame_ids"].astype(int).tolist()
    source_ids = [int(x) for x in pose["frame_ids"]]
    excluded_ids = sorted(set(source_ids) - set(valid_ids))
    trans = selected["translation_residuals_mm"].astype(float)
    rot = selected["rotation_residuals_deg"].astype(float)
    outlier = selected["outlier_mask"].astype(bool)
    outlier_ids = selected["outlier_frame_ids"].astype(int).tolist()

    with (final / "handeye_method_summary.csv").open(
        "r", encoding="utf-8"
    ) as stream:
        rows = list(csv.DictReader(stream))
    method_table = [
        "| 算法 | 有效 | 平均平移残差 (mm) | 平均旋转残差 (deg) | 异常帧数 | 选中 |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        method_table.append(
            "| {method} | {valid} | {mean_translation_residual_mm} | "
            "{mean_rotation_residual_deg} | {outlier_count} | {selected} |".format(
                **row
            )
        )

    residual_table = [
        "| Frame | 平移残差 (mm) | 旋转残差 (deg) | 异常 |",
        "|---:|---:|---:|---:|",
    ]
    for frame, t, r, flag in zip(valid_ids, trans, rot, outlier):
        residual_table.append(
            "| {} | {:.9f} | {:.9f} | {} |".format(
                frame, t, r, "是" if flag else "否"
            )
        )

    calibration_report = reports / "calibration_report.md"
    calibration_report.write_text(
        """# RamanSeg {dataset} Gemini 手眼标定报告

## 结论

- 输入帧数：`{source_count}`；有效标定帧数：`{valid_count}`。
- 使用的标定帧：`{valid_ids}`。
- 排除的标定帧：`{excluded_ids}`（仅因棋盘格未检出，不影响点云融合）。
- 最终算法：`{method}`；矩阵方向：`T_cam2gripper`。
- 固定靶标平移残差：平均 `{mean_t:.9f} mm`，中位数 `{median_t:.9f} mm`，最大 `{max_t:.9f} mm`。
- 固定靶标旋转残差：平均 `{mean_r:.9f} deg`，中位数 `{median_r:.9f} deg`，最大 `{max_r:.9f} deg`。
- MAD 异常帧：`{outlier_ids}`，共 `{outlier_count}` 帧。

## T_cam2gripper

{matrix}

## 坐标和输入核验

- 棋盘格：`pattern_size=(11, 8)`，`square_size=0.015 m`，固定靶标。
- RGB 内参和 OpenCV 顺序畸变直接读取本数据集 `camera_info.json`，没有重估内参。
- 机器人位姿为活动 TCP 的 `T_gripper2base`；Tool ID=1，User Frame ID=0；平移 mm→m。
- RPY 采用已验证的 JAKA `rpy_to_rot_matrix` 约定 `Rz(rz)@Ry(ry)@Rx(rx)`。
- `RunRobotWorldHandEye=False`；保留原始 `create_object_points()` 和 `solvePnP` 流程。
- 位姿旋转最大 `|R.T@R-I|={orth:.3e}`，det(R) 范围 `[{det_min:.15f}, {det_max:.15f}]`。

## 五种算法

{method_table}

选择规则：在合法刚体结果中取固定靶标平均平移残差最小者，旋转残差用于同值决胜。

## 逐帧残差

{residual_table}

## 标定输出绝对路径

{outputs}

- 本报告：`{report}`
""".format(
            dataset=args.dataset,
            source_count=len(source_ids), valid_count=len(valid_ids),
            valid_ids=valid_ids, excluded_ids=excluded_ids, method=method,
            mean_t=float(np.mean(trans)), median_t=float(np.median(trans)),
            max_t=float(np.max(trans)), mean_r=float(np.mean(rot)),
            median_r=float(np.median(rot)), max_r=float(np.max(rot)),
            outlier_ids=outlier_ids, outlier_count=len(outlier_ids),
            matrix=matrix_markdown(T),
            orth=pose["max_rotation_orthogonality_error"],
            det_min=pose["rotation_determinant_min"],
            det_max=pose["rotation_determinant_max"],
            method_table="\n".join(method_table),
            residual_table="\n".join(residual_table),
            outputs=path_list(sorted(p for p in calibration.rglob("*") if p.is_file())),
            report=calibration_report,
        ),
        encoding="utf-8",
    )

    transformed = sorted((processed / "per_frame_base").glob("*.ply"))
    transformed_ids = [int(p.name.split("frame-")[1].split(".")[0]) for p in transformed]
    if transformed_ids != source_ids:
        raise ValueError("Transformed frame IDs do not match source IDs")
    merged = Path(fusion["merged_output"])
    fusion_report = reports / "fusion_report.md"
    fusion_report.write_text(
        """# RamanSeg {dataset} Gemini 彩色点云融合报告

## 结论

- 实际融合帧数：`{count}`；帧号：`{ids}`。
- 排除融合帧：`{skipped}`；棋盘格检测失败帧仍参与融合。
- 输入点数：`{input_points}`；每帧 2 mm 降采样后合计：`{downsampled}`；最终点数：`{output_points}`。
- XYZ 最小边界 (m)：`{xyz_min}`。
- XYZ 最大边界 (m)：`{xyz_max}`。
- XYZ 范围 (m)：`{xyz_range}`。

## 配置和坐标链

- 输入彩色点云位于 RGB Camera 坐标系，单位 m；没有重复执行 D2C 外参。
- `T_cam2base=T_gripper2base@T_cam2gripper`。
- `voxel_size=0.002 m`，`merge_cleanup_interval=5`，`frame_offset=0`，无排除帧。
- 每帧 Base 坐标系点云已保存。

## 输出绝对路径

{outputs}

- 融合统计：`{stats}`
- 本报告：`{report}`
""".format(
            dataset=args.dataset, count=fusion["processed_frame_count"],
            ids=fusion["processed_frame_ids"], skipped=fusion["skipped_frame_ids"],
            input_points=fusion["total_input_points"],
            downsampled=fusion["total_per_frame_points_after_downsampling"],
            output_points=fusion["final_merged_points"],
            xyz_min=fusion["xyz_min_m"], xyz_max=fusion["xyz_max_m"],
            xyz_range=fusion["xyz_range_m"],
            outputs=path_list(transformed + [merged]), stats=fusion_path,
            report=fusion_report,
        ),
        encoding="utf-8",
    )
    print("Calibration report:", calibration_report)
    print("Fusion report:", fusion_report)


if __name__ == "__main__":
    main()
