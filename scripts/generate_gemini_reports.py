#!/usr/bin/env python3

"""Generate final Markdown reports from completed calibration and fusion."""

import csv
import json
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
CALIBRATION = ROOT / "calibration/handeye_gemini"
FINAL = CALIBRATION / "FinalTransforms"
REPORTS = ROOT / "reports/handeye_gemini"
PROCESSED = ROOT / "data/processed"


def matrix_markdown(matrix):
    rows = []
    for row in matrix:
        rows.append("[" + ", ".join("{:.12f}".format(v) for v in row) + "]")
    return "```text\n" + "\n".join(rows) + "\n```"


def list_paths(paths):
    return "\n".join("- `{}`".format(path.resolve()) for path in paths)


def main():
    REPORTS.mkdir(parents=True, exist_ok=True)
    selected_path = FINAL / "T_cam2gripper_selected.npz"
    pose_report_path = REPORTS / "pose_validation.json"
    fusion_stats_path = REPORTS / "fusion_stats.json"
    if not selected_path.is_file():
        raise FileNotFoundError(selected_path)
    if not pose_report_path.is_file():
        raise FileNotFoundError(pose_report_path)
    if not fusion_stats_path.is_file():
        raise FileNotFoundError(fusion_stats_path)

    selected = np.load(selected_path)
    matrix = np.asarray(selected["T"], dtype=np.float64)
    method = str(selected["method"])
    valid_ids = selected["valid_frame_ids"].astype(int).tolist()
    excluded_ids = sorted(set(range(1, 76)) - set(valid_ids))
    translation = selected["translation_residuals_mm"].astype(float)
    rotation = selected["rotation_residuals_deg"].astype(float)
    outlier_mask = selected["outlier_mask"].astype(bool)
    outlier_ids = selected["outlier_frame_ids"].astype(int).tolist()

    with pose_report_path.open("r", encoding="utf-8") as stream:
        pose_report = json.load(stream)
    with fusion_stats_path.open("r", encoding="utf-8") as stream:
        fusion = json.load(stream)

    method_rows = []
    with (FINAL / "handeye_method_summary.csv").open(
        "r", encoding="utf-8"
    ) as stream:
        method_rows = list(csv.DictReader(stream))
    method_table = [
        "| 算法 | 有效 | 平均平移残差 (mm) | 平均旋转残差 (deg) | 异常帧数 | 选中 |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in method_rows:
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
    for frame, trans, rot, outlier in zip(
        valid_ids, translation, rotation, outlier_mask
    ):
        residual_table.append(
            "| {} | {:.9f} | {:.9f} | {} |".format(
                frame, trans, rot, "是" if outlier else "否"
            )
        )

    calibration_outputs = sorted(
        path for path in CALIBRATION.rglob("*") if path.is_file()
    )
    support_outputs = sorted(
        path for path in REPORTS.glob("*") if path.is_file()
    )
    calibration_report_path = REPORTS / "calibration_report.md"
    calibration_text = """# Gemini 336L 手眼标定报告

## 结论

- 最终算法：`{method}`。
- 最终矩阵方向：`T_cam2gripper`（RGB Camera → 采集期间活动 TCP / Gripper）。
- 标定输入：74 帧；棋盘格固定；`pattern_size=(11, 8)`；`square_size=0.015 m`。
- 标定帧：`{valid_ids}`。
- 排除的标定帧：`{excluded_ids}`。Frame 42 仅因棋盘格检测失败而从标定排除。
- 平均固定靶标平移残差：`{mean_t:.9f} mm`；中位数 `{median_t:.9f} mm`；最大值 `{max_t:.9f} mm`。
- 平均固定靶标旋转残差：`{mean_r:.9f} deg`；中位数 `{median_r:.9f} deg`；最大值 `{max_r:.9f} deg`。
- MAD 异常帧：`{outliers}`；共 `{outlier_count}` 帧。

## T_cam2gripper

{matrix}

## 输入与坐标约定核验

- Gemini RGB 内参直接读取 `data/raw/camera_info.json`；未调用 `cv2.calibrateCamera`，未读取旧 `IntrinsicMatrix.npz`。
- RGB 分辨率为 1280×720；畸变顺序使用 `color_camera.distortion.opencv_order`。
- 机器人位置读取 `robot.tcp_pose_raw=[x,y,z,rx,ry,rz]`；平移由 mm 转为 m；RPY 保持 rad。
- Tool ID=1、User Frame ID=0；Gripper Frame 就是采集期间活动 TCP；未转换到法兰，也未使用未经 SDK 验证的平板 Tool Offset。
- RPY 使用 JAKA SDK `rpy_to_rot_matrix` 约定：`R=Rz(rz)@Ry(ry)@Rx(rx)`。用 JAKA SDK 手册参考样例复核的最大元素误差为 `{sdk_error:.3e}`。
- 75 个旋转矩阵最大 `|R.T@R-I|` 为 `{orth_error:.3e}`；det(R) 范围为 `[{det_min:.15f}, {det_max:.15f}]`。
- 机器人位姿方向：`T_gripper2base`；相机位姿由原程序 `solvePnP` 得到 `T_target2cam`。
- 手眼固定靶标链：`T_target2base=T_gripper2base@T_cam2gripper@T_target2cam`。
- `RunRobotWorldHandEye=False`。

JAKA SDK 约定依据：[JAKA Python SDK 的 `rpy_to_rot_matrix`](https://www.jaka.com/docs/documents/SDK/en/python.html)；本机当前未安装可导入的 `jkrc.so/libjakaAPI.so`，因此使用该 SDK 接口的已验证等价矩阵表达式，并以官方数值样例作启动自检。

## 五种算法结果

{method_table}

选择规则：在刚体矩阵有效的结果中，按固定靶标平均平移残差最小选择，平均旋转残差用于同值决胜。

## 逐帧固定靶标残差

{residual_table}

## 标定输出文件（绝对路径）

{calibration_outputs}

## 日志与机器可读统计（绝对路径）

{support_outputs}

- 本报告：`{report_path}`
- 适配标定脚本：`{script_path}`
- 原始标定脚本（未覆盖）：`{original_path}`
""".format(
        method=method,
        valid_ids=valid_ids,
        excluded_ids=excluded_ids,
        mean_t=float(np.mean(translation)),
        median_t=float(np.median(translation)),
        max_t=float(np.max(translation)),
        mean_r=float(np.mean(rotation)),
        median_r=float(np.median(rotation)),
        max_r=float(np.max(rotation)),
        outliers=outlier_ids,
        outlier_count=len(outlier_ids),
        matrix=matrix_markdown(matrix),
        sdk_error=pose_report["jaka_reference_max_abs_error"],
        orth_error=pose_report["max_rotation_orthogonality_error"],
        det_min=pose_report["rotation_determinant_min"],
        det_max=pose_report["rotation_determinant_max"],
        method_table="\n".join(method_table),
        residual_table="\n".join(residual_table),
        calibration_outputs=list_paths(calibration_outputs),
        support_outputs=list_paths(support_outputs),
        report_path=calibration_report_path.resolve(),
        script_path=(ROOT / "scripts/HandEyeCalibration_class_fixed.py").resolve(),
        original_path=Path(
            "/home/root01/HandEyeCalibration/HandEyeCalibration-using-OpenCV/"
            "HandEyeCalibration_class_fixed.py"
        ),
    )
    calibration_report_path.write_text(calibration_text, encoding="utf-8")

    transformed = sorted((PROCESSED / "per_frame_base").glob("*.ply"))
    transformed_ids = [
        int(path.name.split("frame-")[1].split(".")[0]) for path in transformed
    ]
    if transformed_ids != list(range(1, 76)):
        raise ValueError(
            "Transformed per-frame outputs are not exactly frames 1..75"
        )
    merged_path = Path(fusion["merged_output"])
    if not merged_path.is_file():
        raise FileNotFoundError(merged_path)

    fusion_outputs = transformed + [merged_path, fusion_stats_path]
    fusion_report_path = REPORTS / "fusion_report.md"
    fusion_text = """# RamanSeg Gemini 彩色点云融合报告

## 结论

- 实际融合帧数：`{frame_count}`；帧号：`{frame_ids}`。
- 排除融合帧：`{skipped}`。Frame 42 已正常参与融合。
- 输入点总数：`{input_points}`。
- 每帧 2 mm 体素降采样后点数合计：`{frame_downsampled}`。
- 最终输出点数：`{output_points}`。
- XYZ 最小边界 (m)：`{xyz_min}`。
- XYZ 最大边界 (m)：`{xyz_max}`。
- XYZ 范围 (m)：`{xyz_range}`。

## 融合配置与坐标链

- 输入：`frame-*.pointcloud_color_rgb_correct.ply`，单位 m，坐标系为 RGB Camera。
- 变换：`T_cam2base=T_gripper2base@T_cam2gripper`。
- aligned depth 已经 D2C 对齐；没有再次应用 Depth-to-Color 外参。
- `cloud_unit=m`，`voxel_size=0.002 m`，`merge_cleanup_interval=5`，`frame_offset=0`。
- `exclude_frames=[]`；`save_transformed_frames=True`。
- 每一帧转换后点云均保存到 Robot Base 坐标系目录。

## 输出文件（绝对路径）

{fusion_outputs}

## 运行日志（绝对路径）

- `{fusion_log}`

- 本报告：`{report_path}`
- 适配融合脚本：`{script_path}`
- 原始融合脚本（未覆盖）：`{original_path}`
""".format(
        frame_count=fusion["processed_frame_count"],
        frame_ids=fusion["processed_frame_ids"],
        skipped=fusion["skipped_frame_ids"],
        input_points=fusion["total_input_points"],
        frame_downsampled=fusion[
            "total_per_frame_points_after_downsampling"
        ],
        output_points=fusion["final_merged_points"],
        xyz_min=fusion["xyz_min_m"],
        xyz_max=fusion["xyz_max_m"],
        xyz_range=fusion["xyz_range_m"],
        fusion_outputs=list_paths(fusion_outputs),
        fusion_log=(REPORTS / "fusion_run.log").resolve(),
        report_path=fusion_report_path.resolve(),
        script_path=(ROOT / "scripts/transform_and_merge_pointclouds.py").resolve(),
        original_path=Path(
            "/home/root01/HandEyeCalibration/HandEyeCalibration-using-OpenCV/"
            "transform_and_merge_pointclouds.py"
        ),
    )
    fusion_report_path.write_text(fusion_text, encoding="utf-8")

    print("Calibration report:", calibration_report_path)
    print("Fusion report:", fusion_report_path)


if __name__ == "__main__":
    main()
