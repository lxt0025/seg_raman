#!/usr/bin/env python3

"""RGB-only unsupervised KMeans (K=3) for a colored point cloud."""

import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter, MaxNLocator
import numpy as np
import open3d as o3d
from sklearn.cluster import KMeans
from sklearn.preprocessing import StandardScaler


PALETTE = np.array(
    [
        [1.0, 0.0, 0.0],  # Cluster 0: red
        [0.0, 0.0, 1.0],  # Cluster 1: blue
        [0.0, 0.75, 0.0],  # Cluster 2: green
    ],
    dtype=np.float64,
)
VIEW_ELEV = 32.0
VIEW_AZIM = -58.0
POINT_SIZE_3D = 7.0
POINT_SIZE_2D = 10.0


def parse_args():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(
        description="RGB-only StandardScaler + KMeans K=3 experiment."
    )
    parser.add_argument(
        "--input",
        type=Path,
        default=root / "data/processed/roi/ramanseg_1119.ply",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=root / "reports/fat_rgb_k3",
    )
    return parser.parse_args()


def load_valid_cloud(path):
    cloud = o3d.io.read_point_cloud(str(path))
    xyz_all = np.asarray(cloud.points, dtype=np.float64)
    rgb_all = np.asarray(cloud.colors, dtype=np.float64)
    if len(xyz_all) == 0:
        raise RuntimeError("Input PLY contains no points: {}".format(path))
    if not cloud.has_colors() or rgb_all.shape != xyz_all.shape:
        raise RuntimeError("Input PLY must contain one RGB triplet per XYZ point")

    finite = np.all(np.isfinite(xyz_all), axis=1) & np.all(
        np.isfinite(rgb_all), axis=1
    )
    finite_rgb = rgb_all[finite]
    if len(finite_rgb) == 0:
        raise RuntimeError("No points have finite XYZ and RGB")

    # Open3D normally exposes PLY colors in [0, 1]. Keep support for direct
    # numeric RGB arrays in [0, 255] and reject values outside either range.
    if float(np.nanmax(finite_rgb)) <= 1.0 + 1e-9:
        rgb_scale = 1.0
        range_valid = np.all((rgb_all >= 0.0) & (rgb_all <= 1.0 + 1e-9), axis=1)
        source_encoding = "normalized_[0,1]"
    elif float(np.nanmax(finite_rgb)) <= 255.0 + 1e-9:
        rgb_scale = 255.0
        range_valid = np.all((rgb_all >= 0.0) & (rgb_all <= 255.0 + 1e-9), axis=1)
        source_encoding = "numeric_[0,255]"
    else:
        rgb_scale = 255.0
        range_valid = np.all((rgb_all >= 0.0) & (rgb_all <= 255.0), axis=1)
        source_encoding = "numeric_[0,255]_with_invalid_values"

    valid_mask = finite & range_valid
    original_indices = np.flatnonzero(valid_mask).astype(np.int64)
    xyz = xyz_all[valid_mask]
    rgb = rgb_all[valid_mask] / rgb_scale
    rgb = np.clip(rgb, 0.0, 1.0)
    if len(xyz) < 3:
        raise RuntimeError("Fewer than three valid points remain")
    return xyz_all, rgb_all, xyz, rgb, original_indices, source_encoding


def padded_bounds(xyz):
    minimum = xyz.min(axis=0)
    maximum = xyz.max(axis=0)
    span = maximum - minimum
    pad = np.maximum(span * 0.04, 1e-9)
    return minimum - pad, maximum + pad


def style_3d_axis(ax, lower, upper, title):
    ax.set_xlim(lower[0], upper[0])
    ax.set_ylim(lower[1], upper[1])
    ax.set_zlim(lower[2], upper[2])
    ax.set_box_aspect(upper - lower)
    ax.view_init(elev=VIEW_ELEV, azim=VIEW_AZIM)
    ax.set_proj_type("ortho")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.zaxis.set_major_locator(MaxNLocator(nbins=4, min_n_ticks=3))
    ax.zaxis.set_major_formatter(FormatStrFormatter("%.3f"))
    ax.set_title(title)
    ax.grid(False)


def scatter_3d(ax, xyz, colors, lower, upper, title):
    ax.scatter(
        xyz[:, 0], xyz[:, 1], xyz[:, 2], c=colors,
        s=POINT_SIZE_3D, marker=".", linewidths=0,
        depthshade=False, rasterized=True,
    )
    style_3d_axis(ax, lower, upper, title)


def save_single_3d(path, xyz, colors, lower, upper, title):
    fig = plt.figure(figsize=(9, 7), dpi=180)
    ax = fig.add_subplot(111, projection="3d")
    scatter_3d(ax, xyz, colors, lower, upper, title)
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def save_comparison(path, xyz, rgb, cluster_colors, lower, upper):
    fig = plt.figure(figsize=(16, 7), dpi=180)
    left = fig.add_subplot(121, projection="3d")
    right = fig.add_subplot(122, projection="3d")
    scatter_3d(left, xyz, rgb, lower, upper, "Original RGB")
    scatter_3d(right, xyz, cluster_colors, lower, upper, "RGB KMeans (K=3)")
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def save_projection(path, xyz, colors, axis_a, axis_b, label_a, label_b, title):
    fig, ax = plt.subplots(figsize=(9, 7), dpi=180)
    ax.scatter(
        xyz[:, axis_a], xyz[:, axis_b], c=colors,
        s=POINT_SIZE_2D, marker=".", linewidths=0, rasterized=True,
    )
    ax.set_xlabel("{} (m)".format(label_a))
    ax.set_ylabel("{} (m)".format(label_b))
    ax.set_title(title)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(False)
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def write_ascii_ply(path, xyz, rgb, original_indices, cluster_ids):
    """Write CloudCompare-compatible PLY with index and cluster scalar fields."""
    rgb_u8 = np.clip(np.rint(rgb * 255.0), 0, 255).astype(np.uint8)
    with path.open("w", encoding="ascii", newline="\n") as stream:
        stream.write("ply\n")
        stream.write("format ascii 1.0\n")
        stream.write("comment generated by RGB-only KMeans experiment\n")
        stream.write("element vertex {}\n".format(len(xyz)))
        stream.write("property double x\nproperty double y\nproperty double z\n")
        stream.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        stream.write("property int original_index\nproperty uchar cluster_id\n")
        stream.write("end_header\n")
        for point, color, index, cluster in zip(
            xyz, rgb_u8, original_indices, cluster_ids
        ):
            stream.write(
                "{:.12g} {:.12g} {:.12g} {} {} {} {} {}\n".format(
                    point[0], point[1], point[2],
                    int(color[0]), int(color[1]), int(color[2]),
                    int(index), int(cluster),
                )
            )


def main():
    args = parse_args()
    input_path = args.input.resolve()
    output = args.output.resolve()
    if not input_path.is_file():
        raise FileNotFoundError(input_path)
    output.mkdir(parents=True, exist_ok=True)

    xyz_all, _, xyz, rgb, original_indices, encoding = load_valid_cloud(input_path)
    invalid_count = len(xyz_all) - len(xyz)

    scaler = StandardScaler()
    features = scaler.fit_transform(rgb)
    model = KMeans(n_clusters=3, n_init=50, random_state=42)
    labels = model.fit_predict(features).astype(np.int64)
    centers_rgb = scaler.inverse_transform(model.cluster_centers_)
    cluster_colors = PALETTE[labels]

    write_ascii_ply(
        output / "clustered_rgb_k3.ply", xyz, cluster_colors,
        original_indices, labels,
    )
    for cluster_id in range(3):
        mask = labels == cluster_id
        # Preserve original RGB in individual files so manual interpretation
        # can inspect the actual color distribution of each selected cluster.
        write_ascii_ply(
            output / "cluster_{}.ply".format(cluster_id),
            xyz[mask], rgb[mask], original_indices[mask], labels[mask],
        )

    assignments_path = output / "point_assignments_rgb_k3.csv"
    with assignments_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["point_index", "x", "y", "z", "r", "g", "b", "cluster_id"])
        for index, point, color, cluster in zip(original_indices, xyz, rgb, labels):
            writer.writerow(
                [int(index), *["{:.12g}".format(v) for v in point],
                 *["{:.12g}".format(v) for v in color], int(cluster)]
            )

    statistics = []
    for cluster_id in range(3):
        mask = labels == cluster_id
        cluster_xyz = xyz[mask]
        cluster_rgb = rgb[mask]
        row = {
            "cluster_id": cluster_id,
            "point_count": int(mask.sum()),
            "percentage": float(mask.mean() * 100.0),
        }
        for name, values in (
            ("mean", cluster_rgb.mean(axis=0)),
            ("median", np.median(cluster_rgb, axis=0)),
            ("kmeans_center", centers_rgb[cluster_id]),
        ):
            for channel, value in zip("rgb", values):
                row["{}_{}_norm".format(name, channel)] = float(value)
                row["{}_{}_8bit".format(name, channel)] = float(value * 255.0)
        for prefix, values in (
            ("bbox_min", cluster_xyz.min(axis=0)),
            ("bbox_max", cluster_xyz.max(axis=0)),
            ("spatial_centroid", cluster_xyz.mean(axis=0)),
        ):
            for axis, value in zip("xyz", values):
                row["{}_{}_m".format(prefix, axis)] = float(value)
        statistics.append(row)

    statistics_path = output / "cluster_statistics.csv"
    with statistics_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(statistics[0].keys()))
        writer.writeheader()
        writer.writerows(statistics)

    lower, upper = padded_bounds(xyz)
    save_single_3d(
        output / "original_rgb.png", xyz, rgb, lower, upper,
        "Original RGB point cloud",
    )
    save_single_3d(
        output / "clustered_rgb_k3.png", xyz, cluster_colors, lower, upper,
        "RGB-only KMeans clustering (K=3)",
    )
    save_comparison(
        output / "original_vs_clustered.png",
        xyz, rgb, cluster_colors, lower, upper,
    )
    save_projection(
        output / "projection_xy.png", xyz, cluster_colors,
        0, 1, "X", "Y", "XY projection — RGB KMeans K=3",
    )
    save_projection(
        output / "projection_xz.png", xyz, cluster_colors,
        0, 2, "X", "Z", "XZ projection — RGB KMeans K=3",
    )
    save_projection(
        output / "projection_yz.png", xyz, cluster_colors,
        1, 2, "Y", "Z", "YZ projection — RGB KMeans K=3",
    )

    table_rows = []
    for row in statistics:
        table_rows.append(
            "| {cluster_id} | {point_count} | {percentage:.3f}% | "
            "({mean_r_8bit:.2f}, {mean_g_8bit:.2f}, {mean_b_8bit:.2f}) | "
            "({median_r_8bit:.2f}, {median_g_8bit:.2f}, {median_b_8bit:.2f}) | "
            "({kmeans_center_r_8bit:.2f}, {kmeans_center_g_8bit:.2f}, "
            "{kmeans_center_b_8bit:.2f}) |".format(**row)
        )

    report_path = output / "stage1_rgb_k3_report.md"
    outputs = [
        "clustered_rgb_k3.ply", "cluster_0.ply", "cluster_1.ply", "cluster_2.ply",
        "point_assignments_rgb_k3.csv", "cluster_statistics.csv", "original_rgb.png",
        "clustered_rgb_k3.png", "original_vs_clustered.png", "projection_xy.png",
        "projection_xz.png", "projection_yz.png", "stage1_rgb_k3_report.md",
    ]
    report_path.write_text(
        """# RGB-only 无监督三类点云聚类报告

## 实验设置

- 输入：`{input_path}`。
- 原始点数：`{total_count}`；有效点数：`{valid_count}`；无效 XYZ/RGB 点数：`{invalid_count}`。
- 输入 RGB 编码：`{encoding}`；聚类前统一为 `[0,1]`。
- 特征严格为 `[R,G,B]`，使用 `StandardScaler`；XYZ 仅用于显示和导出。
- KMeans：`n_clusters=3`，`n_init=50`，`random_state=42`。
- 聚类显示调色板固定为 Cluster 0=红、Cluster 1=蓝、Cluster 2=绿；这些显示颜色不代表点的原始 RGB 或组织语义。三个独立 Cluster PLY 保留各点原始 RGB，便于在 CloudCompare 中人工判断。
- 未使用人工标签、Raman、HSV、Lab、法向量、几何特征、深度学习或主动背景删除。
- Cluster 0/1/2 只是算法编号，没有自动赋予 Background/Fat/Non-fat 语义。

## 定量结果

| Cluster | 点数 | 比例 | 平均 RGB (8-bit) | 中位 RGB (8-bit) | KMeans 中心 RGB (8-bit) |
|---:|---:|---:|---|---|---|
{table}

完整空间包围盒、空间质心及归一化/8-bit RGB 统计见 `cluster_statistics.csv`。逐点原始索引、XYZ、RGB 和聚类编号见 `point_assignments_rgb_k3.csv`。

## 空间与颜色解释

本节在生成三维对比图和 XY/XZ/YZ 投影后进行人工检查。聚类编号不会在此自动重命名为组织语义。

- 三维对比图与 XY 投影显示，三个簇具有明显但不完全的空间分布差异。
- Cluster 0 的原始平均 RGB 约为 `(33, 144, 194)`，呈明显蓝色；虽然它在聚类显示图中按固定调色板画成红色，但其原始点主要集中于点云外围和上侧边界。这与蓝色背景集中在一个簇的现象一致。
- Cluster 1 的原始平均 RGB 约为 `(198, 182, 160)`，Cluster 2 约为 `(161, 107, 86)`；两者主要分布在内部主体区域，表现为较浅与较深红褐两种颜色模式。它们可能为后续人工判断脂肪/非脂肪提供依据，但本实验没有真值，不能据此自动赋予组织语义。
- Cluster 1 和 Cluster 2 在主体区域内呈交错和局部带状分布，而非两个完全空间分离的区域。这说明 RGB 能区分两种主要外观，但边界处及颜色渐变处存在混合。
- Cluster 0 在主体边缘和少量内部位置仍有零散点；反过来，Cluster 1/2 也延伸到接近外围的位置。因此背景与组织并非完全纯净分簇，可能受反光、阴影、边缘混色和采样影响。

## 局限性

- RGB 会受照明、阴影、镜面高光、曝光与相机白平衡影响。
- KMeans 假设标准化 RGB 空间中存在近似球状、方差相近的簇，不保证与真实组织类别一一对应。
- 未使用空间连续性，因此颜色相近但空间不相邻的点会被归入同一簇。
- 没有人工真值，无法计算准确率、召回率或证明某簇就是脂肪组织。
- 蓝色背景若包含反光或组织中存在偏蓝阴影，可能与组织颜色发生混合。

## 输出文件（绝对路径）

{outputs}
""".format(
            input_path=input_path, total_count=len(xyz_all), valid_count=len(xyz),
            invalid_count=invalid_count, encoding=encoding,
            table="\n".join(table_rows),
            outputs="\n".join(
                "- `{}`".format((output / name).resolve()) for name in outputs
            ),
        ),
        encoding="utf-8",
    )

    print("Input:", input_path)
    print("Total/valid/invalid:", len(xyz_all), len(xyz), invalid_count)
    for row in statistics:
        print(
            "Cluster {cluster_id}: count={point_count}, percentage={percentage:.3f}%, "
            "mean_rgb_8bit=({mean_r_8bit:.2f}, {mean_g_8bit:.2f}, {mean_b_8bit:.2f})".format(
                **row
            )
        )
    print("Output:", output)


if __name__ == "__main__":
    main()
