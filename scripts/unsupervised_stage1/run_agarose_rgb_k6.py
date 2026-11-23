#!/usr/bin/env python3

"""RGB-only unsupervised KMeans K=6 for the cropped agarose cloud."""

import argparse
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import FormatStrFormatter, MaxNLocator
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler


PALETTE = np.array(
    [
        [1.00, 0.00, 0.00],  # 0 red
        [0.00, 0.20, 1.00],  # 1 blue
        [0.00, 0.70, 0.00],  # 2 green
        [1.00, 0.50, 0.00],  # 3 orange
        [0.60, 0.10, 0.80],  # 4 purple
        [0.00, 0.85, 0.85],  # 5 cyan
    ],
    dtype=np.float64,
)
VIEW_ELEV = 34.0
VIEW_AZIM = -58.0
POINT_SIZE_3D = 11.0
POINT_SIZE_2D = 14.0


def parse_args():
    root = Path(__file__).resolve().parents[2]
    parser = argparse.ArgumentParser(
        description="RGB-only StandardScaler + KMeans K=6 for agarose."
    )
    parser.add_argument(
        "--input", type=Path,
        default=root / "data/processed/roi/ramanseg_1120.ply",
    )
    parser.add_argument(
        "--output", type=Path,
        default=root / "reports/agarose_rgb_k6",
    )
    return parser.parse_args()


def load_valid_cloud(path):
    cloud = o3d.io.read_point_cloud(str(path))
    xyz_all = np.asarray(cloud.points, dtype=np.float64)
    rgb_all = np.asarray(cloud.colors, dtype=np.float64)
    if len(xyz_all) == 0:
        raise RuntimeError("Input PLY contains no XYZ points")
    if not cloud.has_colors() or rgb_all.shape != xyz_all.shape:
        raise RuntimeError("Input PLY must contain one RGB triplet per XYZ point")

    finite = np.all(np.isfinite(xyz_all), axis=1) & np.all(
        np.isfinite(rgb_all), axis=1
    )
    finite_rgb = rgb_all[finite]
    if len(finite_rgb) == 0:
        raise RuntimeError("No finite XYZ/RGB points remain")

    if float(np.max(finite_rgb)) <= 1.0 + 1e-9:
        scale = 1.0
        in_range = np.all((rgb_all >= 0.0) & (rgb_all <= 1.0 + 1e-9), axis=1)
        encoding = "normalized_[0,1]"
    else:
        scale = 255.0
        in_range = np.all((rgb_all >= 0.0) & (rgb_all <= 255.0 + 1e-9), axis=1)
        encoding = "numeric_[0,255]"

    valid = finite & in_range
    original_indices = np.flatnonzero(valid).astype(np.int64)
    xyz = xyz_all[valid]
    rgb = np.clip(rgb_all[valid] / scale, 0.0, 1.0)
    if len(xyz) < 6:
        raise RuntimeError("Fewer than six valid points remain")
    return xyz_all, xyz, rgb, original_indices, encoding


def write_ascii_ply(path, xyz, rgb, original_indices, labels):
    rgb_u8 = np.clip(np.rint(rgb * 255.0), 0, 255).astype(np.uint8)
    with path.open("w", encoding="ascii", newline="\n") as stream:
        stream.write("ply\nformat ascii 1.0\n")
        stream.write("comment RGB-only unsupervised KMeans K=6\n")
        stream.write("element vertex {}\n".format(len(xyz)))
        stream.write("property double x\nproperty double y\nproperty double z\n")
        stream.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        stream.write("property int original_index\nproperty uchar cluster_id\n")
        stream.write("end_header\n")
        for point, color, index, label in zip(xyz, rgb_u8, original_indices, labels):
            stream.write(
                "{:.12g} {:.12g} {:.12g} {} {} {} {} {}\n".format(
                    point[0], point[1], point[2],
                    int(color[0]), int(color[1]), int(color[2]),
                    int(index), int(label),
                )
            )


def padded_bounds(xyz):
    minimum = xyz.min(axis=0)
    maximum = xyz.max(axis=0)
    padding = np.maximum((maximum - minimum) * 0.04, 1e-9)
    return minimum - padding, maximum + padding


def style_3d(ax, lower, upper, title):
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
    style_3d(ax, lower, upper, title)


def save_single_3d(path, xyz, colors, lower, upper, title):
    fig = plt.figure(figsize=(9, 7), dpi=180)
    ax = fig.add_subplot(111, projection="3d")
    scatter_3d(ax, xyz, colors, lower, upper, title)
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def save_comparison(path, xyz, rgb, display_colors, lower, upper):
    fig = plt.figure(figsize=(16, 7), dpi=180)
    left = fig.add_subplot(121, projection="3d")
    right = fig.add_subplot(122, projection="3d")
    scatter_3d(left, xyz, rgb, lower, upper, "Original RGB Point Cloud")
    scatter_3d(
        right, xyz, display_colors, lower, upper,
        "Unsupervised RGB Clustering (K=6)",
    )
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


def save_projection(path, xyz, colors, a, b, a_name, b_name):
    fig, ax = plt.subplots(figsize=(9, 7), dpi=180)
    ax.scatter(
        xyz[:, a], xyz[:, b], c=colors, s=POINT_SIZE_2D,
        marker=".", linewidths=0, rasterized=True,
    )
    ax.set_xlabel("{} (m)".format(a_name))
    ax.set_ylabel("{} (m)".format(b_name))
    ax.set_title("{}{} projection — RGB KMeans K=6".format(a_name, b_name))
    ax.set_aspect("equal", adjustable="box")
    ax.grid(False)
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight")
    plt.close(fig)


class UnionFind:
    def __init__(self, size):
        self.parent = np.arange(size, dtype=np.int64)
        self.rank = np.zeros(size, dtype=np.int8)

    def find(self, item):
        root = item
        while self.parent[root] != root:
            root = self.parent[root]
        while self.parent[item] != item:
            parent = self.parent[item]
            self.parent[item] = root
            item = parent
        return int(root)

    def union(self, a, b):
        root_a = self.find(a)
        root_b = self.find(b)
        if root_a == root_b:
            return
        if self.rank[root_a] < self.rank[root_b]:
            root_a, root_b = root_b, root_a
        self.parent[root_b] = root_a
        if self.rank[root_a] == self.rank[root_b]:
            self.rank[root_a] += 1


def radius_components(points, radius):
    """Return component IDs sorted by descending component size."""
    if len(points) == 0:
        return np.empty(0, dtype=np.int64)
    union = UnionFind(len(points))
    tree = cKDTree(points)
    for a, b in tree.query_pairs(radius):
        union.union(a, b)
    roots = np.array([union.find(i) for i in range(len(points))], dtype=np.int64)
    unique, counts = np.unique(roots, return_counts=True)
    order = unique[np.argsort(-counts)]
    root_to_id = {int(root): component_id for component_id, root in enumerate(order)}
    return np.array([root_to_id[int(root)] for root in roots], dtype=np.int64)


def main():
    args = parse_args()
    input_path = args.input.resolve()
    output = args.output.resolve()
    clusters_dir = output / "clusters"
    if not input_path.is_file():
        raise FileNotFoundError(input_path)
    clusters_dir.mkdir(parents=True, exist_ok=True)

    xyz_all, xyz, rgb, original_indices, encoding = load_valid_cloud(input_path)
    invalid_count = len(xyz_all) - len(xyz)

    scaler = StandardScaler()
    standardized_rgb = scaler.fit_transform(rgb)
    kmeans = KMeans(
        n_clusters=6, n_init=50, random_state=42, max_iter=500
    )
    labels = kmeans.fit_predict(standardized_rgb).astype(np.int64)
    centers_standardized = kmeans.cluster_centers_
    centers_rgb = scaler.inverse_transform(centers_standardized)
    display_colors = PALETTE[labels]

    pca = PCA(n_components=2)
    pca_xy = pca.fit_transform(standardized_rgb)

    # Derive one global interpretation-only radius from original sampling.
    nn_tree = cKDTree(xyz)
    nn_distances, _ = nn_tree.query(xyz, k=2, workers=-1)
    nearest_neighbor = nn_distances[:, 1]
    median_nn = float(np.median(nearest_neighbor))
    p95_nn = float(np.percentile(nearest_neighbor, 95))
    component_radius = max(2.5 * median_nn, 1.25 * p95_nn)

    write_ascii_ply(
        output / "rgb_k6_clustered.ply", xyz, display_colors,
        original_indices, labels,
    )
    for cluster_id in range(6):
        mask = labels == cluster_id
        # Independent clouds preserve original RGB for manual interpretation.
        write_ascii_ply(
            clusters_dir / "cluster_{}.ply".format(cluster_id),
            xyz[mask], rgb[mask], original_indices[mask], labels[mask],
        )

    with (output / "point_assignments_rgb_k6.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(["point_index", "x", "y", "z", "r", "g", "b", "cluster_id"])
        for index, point, color, label in zip(original_indices, xyz, rgb, labels):
            writer.writerow(
                [int(index), *["{:.12g}".format(v) for v in point],
                 *["{:.12g}".format(v) for v in color], int(label)]
            )

    statistics = []
    for cluster_id in range(6):
        mask = labels == cluster_id
        cluster_xyz = xyz[mask]
        cluster_rgb = rgb[mask]
        row = {
            "cluster_id": cluster_id,
            "point_count": int(mask.sum()),
            "point_percentage": float(mask.mean() * 100.0),
        }
        for prefix, values in (
            ("mean", cluster_rgb.mean(axis=0)),
            ("median", np.median(cluster_rgb, axis=0)),
            ("rgb_centroid", centers_rgb[cluster_id]),
            ("standardized_center", centers_standardized[cluster_id]),
        ):
            for channel, value in zip("RGB", values):
                row["{}_{}".format(prefix, channel)] = float(value)
        xyz_min = cluster_xyz.min(axis=0)
        xyz_max = cluster_xyz.max(axis=0)
        for prefix, values in (
            ("spatial_centroid", cluster_xyz.mean(axis=0)),
            ("xyz_min", xyz_min), ("xyz_max", xyz_max),
            ("xyz_extent", xyz_max - xyz_min),
        ):
            for axis, value in zip("XYZ", values):
                row["{}_{}_m".format(prefix, axis)] = float(value)
        statistics.append(row)

    with (output / "cluster_statistics_rgb_k6.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(statistics[0].keys()))
        writer.writeheader()
        writer.writerows(statistics)

    components = []
    for cluster_id in range(6):
        global_mask = labels == cluster_id
        cluster_xyz = xyz[global_mask]
        component_ids = radius_components(cluster_xyz, component_radius)
        for component_id in np.unique(component_ids):
            component = cluster_xyz[component_ids == component_id]
            xyz_min = component.min(axis=0)
            xyz_max = component.max(axis=0)
            row = {
                "cluster_id": cluster_id,
                "component_id": int(component_id),
                "point_count": len(component),
                "point_percentage_within_cluster": float(
                    len(component) / len(cluster_xyz) * 100.0
                ),
            }
            for prefix, values in (
                ("spatial_centroid", component.mean(axis=0)),
                ("xyz_min", xyz_min), ("xyz_max", xyz_max),
                ("xyz_extent", xyz_max - xyz_min),
            ):
                for axis, value in zip("XYZ", values):
                    row["{}_{}_m".format(prefix, axis)] = float(value)
            components.append(row)

    with (output / "cluster_spatial_components.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.DictWriter(stream, fieldnames=list(components[0].keys()))
        writer.writeheader()
        writer.writerows(components)

    with (output / "rgb_k6_pca_coordinates.csv").open(
        "w", encoding="utf-8", newline=""
    ) as stream:
        writer = csv.writer(stream)
        writer.writerow(["point_index", "PC1", "PC2", "cluster_id"])
        for index, coordinates, label in zip(original_indices, pca_xy, labels):
            writer.writerow(
                [int(index), "{:.12g}".format(coordinates[0]),
                 "{:.12g}".format(coordinates[1]), int(label)]
            )

    lower, upper = padded_bounds(xyz)
    save_single_3d(
        output / "original_rgb.png", xyz, rgb, lower, upper,
        "Original RGB Point Cloud",
    )
    save_single_3d(
        output / "rgb_k6_clustered.png", xyz, display_colors, lower, upper,
        "Unsupervised RGB Clustering (K=6)",
    )
    save_comparison(
        output / "original_vs_rgb_k6.png",
        xyz, rgb, display_colors, lower, upper,
    )
    save_projection(output / "rgb_k6_xy.png", xyz, display_colors, 0, 1, "X", "Y")
    save_projection(output / "rgb_k6_xz.png", xyz, display_colors, 0, 2, "X", "Z")
    save_projection(output / "rgb_k6_yz.png", xyz, display_colors, 1, 2, "Y", "Z")

    fig, ax = plt.subplots(figsize=(9, 7), dpi=180)
    ax.scatter(
        pca_xy[:, 0], pca_xy[:, 1], c=display_colors,
        s=POINT_SIZE_2D, marker=".", linewidths=0, rasterized=True,
    )
    variance = pca.explained_variance_ratio_ * 100.0
    ax.set_xlabel("PC1 ({:.2f}% explained variance)".format(variance[0]))
    ax.set_ylabel("PC2 ({:.2f}% explained variance)".format(variance[1]))
    ax.set_title("PCA of standardized RGB features — KMeans K=6")
    ax.grid(False)
    fig.tight_layout()
    fig.savefig(output / "rgb_k6_pca.png", bbox_inches="tight")
    plt.close(fig)

    config = {
        "input_ply": str(input_path),
        "output_directory": str(output),
        "original_point_count": len(xyz_all),
        "valid_point_count": len(xyz),
        "invalid_point_count": invalid_count,
        "rgb_input_encoding": encoding,
        "rgb_normalized_min": rgb.min(axis=0).tolist(),
        "rgb_normalized_max": rgb.max(axis=0).tolist(),
        "features": ["R", "G", "B"],
        "xyz_used_by_kmeans": False,
        "scaler": {
            "type": "StandardScaler",
            "mean": scaler.mean_.tolist(),
            "scale": scaler.scale_.tolist(),
        },
        "kmeans": {
            "n_clusters": 6, "n_init": 50,
            "random_state": 42, "max_iter": 500,
            "inertia": float(kmeans.inertia_),
            "n_iter": int(kmeans.n_iter_),
        },
        "pca_visualization_only": True,
        "pca_explained_variance_percentage": variance.tolist(),
        "spatial_components": {
            "method": "radius_graph_connected_components_within_each_RGB_cluster",
            "radius_m": component_radius,
            "global_nearest_neighbor_median_m": median_nn,
            "global_nearest_neighbor_p95_m": p95_nn,
            "radius_rule": "max(2.5*median_NN, 1.25*p95_NN)",
            "xyz_fed_back_to_kmeans": False,
        },
        "visualization_palette_rgb_normalized": PALETTE.tolist(),
    }
    (output / "run_config.json").write_text(
        json.dumps(config, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    table_rows = []
    for row in statistics:
        table_rows.append(
            "| {cluster_id} | {point_count} | {point_percentage:.3f}% | "
            "({mean_R:.3f}, {mean_G:.3f}, {mean_B:.3f}) | "
            "({median_R:.3f}, {median_G:.3f}, {median_B:.3f}) | "
            "({spatial_centroid_X_m:.4f}, {spatial_centroid_Y_m:.4f}, "
            "{spatial_centroid_Z_m:.4f}) | "
            "({xyz_extent_X_m:.4f}, {xyz_extent_Y_m:.4f}, {xyz_extent_Z_m:.4f}) |".format(
                **row
            )
        )

    component_counts = {
        cluster_id: sum(1 for row in components if row["cluster_id"] == cluster_id)
        for cluster_id in range(6)
    }
    outputs = [
        output / "rgb_k6_clustered.ply",
        *[clusters_dir / "cluster_{}.ply".format(i) for i in range(6)],
        output / "point_assignments_rgb_k6.csv",
        output / "cluster_statistics_rgb_k6.csv",
        output / "cluster_spatial_components.csv",
        output / "original_rgb.png", output / "rgb_k6_clustered.png",
        output / "original_vs_rgb_k6.png", output / "rgb_k6_xy.png",
        output / "rgb_k6_xz.png", output / "rgb_k6_yz.png",
        output / "rgb_k6_pca.png", output / "rgb_k6_pca_coordinates.csv",
        output / "run_config.json", output / "agarose_rgb_k6_report.md",
    ]
    report = output / "agarose_rgb_k6_report.md"
    report.write_text(
        """# Agarose RGB-only 无监督六类点云聚类报告

## 基本信息

- 输入 PLY：`{input_path}`；原文件未修改。
- 原始点数：`{original_count}`；有效点数：`{valid_count}`；无效点数：`{invalid_count}`。
- RGB 归一化范围：min=`{rgb_min}`，max=`{rgb_max}`。
- 特征仅为全局 `[R,G,B]`；使用一次 `StandardScaler`；XYZ 未进入 KMeans。
- KMeans：`n_clusters=6`，`n_init=50`，`random_state=42`，`max_iter=500`。
- PCA 只用于二维显示，PC1/PC2 解释方差分别为 `{pc1:.3f}%`、`{pc2:.3f}%`。
- 空间连通分析半径：`{radius:.6f} m`；仅用于解释，没有反馈给 RGB KMeans。
- 当前输入按用户确认不含背景；Cluster 0~5 只是无监督编号，没有自动赋予浓度含义。

## Cluster 组成

| Cluster | 点数 | 比例 | RGB 均值 | RGB 中位数 | 空间质心 XYZ (m) | XYZ 范围 (m) |
|---:|---:|---:|---|---|---|---|
{table}

标准化 KMeans 中心、逆变换 RGB 中心、XYZ 最小/最大值等完整数值见 `cluster_statistics_rgb_k6.csv`。

## 空间连通分量

按簇统计的半径图连通分量数量：`{component_counts}`。完整分量点数、质心、包围盒与范围见 `cluster_spatial_components.csv`。

## 空间解释

- 当前无背景点云在 XYZ 中表现为一个连续的近矩形采样区域，六个 RGB 簇没有形成六个彼此分开的空间块。
- Cluster 0、Cluster 1 和 Cluster 5 是三个主要颜色簇，合计占全部有效点约 `94.26%`，并在 XY 平面内广泛交错分布，而非各自占据一个独立区域。
- Cluster 2 仅包含 1 个点，Cluster 3 仅包含 11 个点；PCA 中它们对应远离主体颜色分布的极端或稀有 RGB 点。由于实验要求只移除 NaN/Inf 且不手工删除点，这些点被完整保留。
- Cluster 4 也属于较小簇，并散布在多个位置。空间连通分量结果显示多数簇包含多个分离小区域，说明相同 RGB 模式会出现在不同物理位置。
- 单一局部区域通常同时包含 Cluster 0、1、5 及少量其他簇，说明同一物理区域内部存在多种 RGB 模式或颜色连续变化。
- PCA 显示主要点群在 RGB 空间中连续重叠，而少数极端点显著拉长 PC1；因此不能将六个 Cluster 自动解释为六个浓度组。

## 局限性

- 这是无监督 RGB 聚类，没有人工标签，也没有使用 Raman 数据。
- Cluster ID 不具有内在浓度含义，必须结合原始点云空间位置人工解释。
- RGB 差异可能来自照明、反射、曝光、白平衡、材料外观或成像角度。
- `K=6` 来自六个物理浓度组的实验预期，并不表示 KMeans 独立发现了六种化学类别。
- 本实验不能证明已经识别化学浓度，也不能替代有标签的定量验证。
- KMeans 不使用空间连续性；颜色相近的多个物理区域可能共享一个簇，同一物理块也可能含多个簇。

## 输出文件（绝对路径）

{outputs}
""".format(
            input_path=input_path, original_count=len(xyz_all),
            valid_count=len(xyz), invalid_count=invalid_count,
            rgb_min=rgb.min(axis=0).tolist(), rgb_max=rgb.max(axis=0).tolist(),
            pc1=variance[0], pc2=variance[1], radius=component_radius,
            table="\n".join(table_rows), component_counts=component_counts,
            outputs="\n".join("- `{}`".format(path.resolve()) for path in outputs),
        ),
        encoding="utf-8",
    )

    print("Input:", input_path)
    print("Total/valid/invalid:", len(xyz_all), len(xyz), invalid_count)
    print("Spatial component radius/m:", component_radius)
    for row in statistics:
        print(
            "Cluster {cluster_id}: count={point_count}, percentage={point_percentage:.3f}%, "
            "mean_RGB=({mean_R:.3f}, {mean_G:.3f}, {mean_B:.3f}), components={}".format(
                component_counts[row["cluster_id"]], **row
            )
        )
    print("Output:", output)


if __name__ == "__main__":
    main()
