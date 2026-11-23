#!/usr/bin/env python3
"""Plot 2D overlays from an existing K=6 point assignment CSV."""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d
from matplotlib.lines import Line2D


ROOT = Path(__file__).resolve().parents[2]
PLY = ROOT / "data/processed/roi/ramanseg_1120.ply"
ASSIGNMENTS = ROOT / "reports/agarose_rgb_k6/point_assignments_rgb_k6.csv"
OUT = ROOT / "reports/rgb_k6_overlay"
COLORS = np.array([
    [1.0, 0.0, 0.0], [0.0, 0.2, 1.0], [0.0, 0.7, 0.0],
    [1.0, 0.5, 0.0], [0.6, 0.1, 0.8], [0.0, 0.85, 0.85],
])
NAMES = ["red", "blue", "green", "orange", "purple", "cyan"]
BASE_ALPHA = 0.38
OVERLAY_ALPHA = 0.70
BASE_SIZE = 34
OVERLAY_SIZE = 19


def plot_plane(coords, base_rgb, labels, dimensions, axis_names, title, path, mode):
    fig, ax = plt.subplots(figsize=(9, 8), dpi=300)
    a, b = dimensions
    if mode in ("overlay", "base"):
        ax.scatter(coords[:, a], coords[:, b], c=base_rgb, s=BASE_SIZE,
                   alpha=BASE_ALPHA, linewidths=0, rasterized=True, zorder=1)
    if mode in ("overlay", "clusters"):
        for cluster in range(6):
            mask = labels == cluster
            ax.scatter(coords[mask, a], coords[mask, b],
                       c=[COLORS[cluster]], s=OVERLAY_SIZE,
                       alpha=OVERLAY_ALPHA, linewidths=0.25,
                       edgecolors="white", rasterized=True, zorder=2)
    ax.set_xlabel("{} (m)".format(axis_names[0]))
    ax.set_ylabel("{} (m)".format(axis_names[1]))
    ax.set_title(title, fontsize=15, pad=12)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlim(coords[:, a].min(), coords[:, a].max())
    ax.set_ylim(coords[:, b].min(), coords[:, b].max())
    ax.set_facecolor("white")
    fig.patch.set_facecolor("white")
    ax.margins(0.025)
    if mode in ("overlay", "clusters"):
        handles = [Line2D([0], [0], marker="o", color="w",
                          markerfacecolor=COLORS[i], markersize=8,
                          label="Cluster {}".format(i)) for i in range(6)]
        ax.legend(handles=handles, loc="best", frameon=True, fontsize=9)
    fig.tight_layout()
    fig.savefig(path, bbox_inches="tight", facecolor="white")
    plt.close(fig)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    cloud = o3d.io.read_point_cloud(str(PLY))
    xyz = np.asarray(cloud.points, dtype=np.float64)
    rgb = np.asarray(cloud.colors, dtype=np.float64)
    if not cloud.has_colors() or rgb.shape != xyz.shape:
        raise ValueError("Original PLY must have one RGB per XYZ point")
    valid = np.isfinite(xyz).all(axis=1) & np.isfinite(rgb).all(axis=1)
    original_indices = np.flatnonzero(valid)
    xyz = xyz[valid]
    rgb = np.clip(rgb[valid], 0, 1)

    with ASSIGNMENTS.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    required = ["point_index", "x", "y", "z", "r", "g", "b", "cluster_id"]
    if not rows or list(rows[0].keys()) != required:
        raise ValueError("Assignment CSV schema does not match expected columns")
    indices = np.array([int(row["point_index"]) for row in rows])
    csv_xyz = np.array([[float(row[k]) for k in ("x", "y", "z")] for row in rows])
    csv_rgb = np.array([[float(row[k]) for k in ("r", "g", "b")] for row in rows])
    labels = np.array([int(row["cluster_id"]) for row in rows])
    if len(rows) != len(xyz):
        raise ValueError("Valid original count {} != CSV row count {}".format(len(xyz), len(rows)))
    if not np.array_equal(indices, original_indices):
        raise ValueError("point_index sequence does not match valid original PLY order")
    if not np.allclose(csv_xyz, xyz, rtol=0, atol=1e-8):
        raise ValueError("CSV XYZ coordinates do not match original PLY order")
    if not np.allclose(csv_rgb, rgb, rtol=0, atol=1e-6):
        raise ValueError("CSV RGB values do not match original PLY")
    unique = set(np.unique(labels).tolist())
    if unique != set(range(6)):
        raise ValueError("Expected cluster IDs 0..5; found {}".format(sorted(unique)))

    outputs = {
        "xy_overlay": OUT / "rgb_k6_xy_overlay.png",
        "xy_cluster_only": OUT / "rgb_k6_xy_cluster_only.png",
        "xy_base_only": OUT / "rgb_k6_xy_base_only.png",
        "xz_overlay": OUT / "rgb_k6_xz_overlay.png",
        "yz_overlay": OUT / "rgb_k6_yz_overlay.png",
    }
    plot_plane(xyz, rgb, labels, (0, 1), ("X", "Y"),
               "RGB Unsupervised Clustering (K=6) - XY Overlay",
               outputs["xy_overlay"], "overlay")
    plot_plane(xyz, rgb, labels, (0, 1), ("X", "Y"),
               "RGB Clusters (K=6) - XY", outputs["xy_cluster_only"], "clusters")
    plot_plane(xyz, rgb, labels, (0, 1), ("X", "Y"),
               "Original RGB - XY", outputs["xy_base_only"], "base")
    plot_plane(xyz, rgb, labels, (0, 2), ("X", "Z"),
               "RGB Unsupervised Clustering (K=6) - XZ Overlay",
               outputs["xz_overlay"], "overlay")
    plot_plane(xyz, rgb, labels, (1, 2), ("Y", "Z"),
               "RGB Unsupervised Clustering (K=6) - YZ Overlay",
               outputs["yz_overlay"], "overlay")

    config = {
        "original_ply": str(PLY.resolve()),
        "assignments_csv": str(ASSIGNMENTS.resolve()),
        "valid_original_points": len(xyz),
        "assignment_rows": len(rows),
        "point_index_order_valid": True,
        "xyz_match_max_abs_error": float(np.max(np.abs(csv_xyz - xyz))),
        "rgb_match_max_abs_error": float(np.max(np.abs(csv_rgb - rgb))),
        "cluster_ids": sorted(unique),
        "cluster_colors_rgb": {str(i): COLORS[i].tolist() for i in range(6)},
        "projection": "orthographic scatter; XY main, XZ/YZ supplemental",
        "base_alpha": BASE_ALPHA,
        "overlay_alpha": OVERLAY_ALPHA,
        "base_point_size": BASE_SIZE,
        "overlay_point_size": OVERLAY_SIZE,
        "outputs": {key: str(path.resolve()) for key, path in outputs.items()},
    }
    config_path = OUT / "overlay_run_config.json"
    config_path.write_text(json.dumps(config, indent=2), encoding="utf-8")
    report_path = OUT / "overlay_report.md"
    report_path.write_text(
        """# RGB K=6 点云二维叠加可视化报告

- 原始 PLY：`{ply}`
- K=6 聚类 CSV：`{csv}`
- 绘制点数：`{count}`（原始有效点与 CSV 行数一致；索引和 XYZ/RGB 对齐）。
- 投影：正交 XY（X 横轴、Y 纵轴）；另输出 XZ、YZ。
- 底图 alpha：`{base_alpha}`；覆盖层 alpha：`{overlay_alpha}`。
- 底图点大小：`{base_size}`；聚类点大小：`{overlay_size}`。
- 固定簇颜色：{mapping}

本图使用已有 K=6 聚类结果，没有重新运行聚类，也没有更改簇编号。原始 RGB 底图有意降低视觉权重，聚类覆盖层用于突出空间结构，便于观察簇的分布。

XY 底图由原始 PLY 中每个有效点的 `(X,Y)` 正交投影直接绘制，并使用该点原始 RGB；为使底图在同位置的聚类点下仍可见，底图点略大且聚类覆盖点采用半透明较小点绘制。

## 输出文件

{outputs}
""".format(
            ply=PLY.resolve(), csv=ASSIGNMENTS.resolve(), count=len(xyz),
            base_alpha=BASE_ALPHA, overlay_alpha=OVERLAY_ALPHA,
            base_size=BASE_SIZE, overlay_size=OVERLAY_SIZE,
            mapping=", ".join("Cluster {}: {}".format(i, NAMES[i]) for i in range(6)),
            outputs="\n".join("- `{}`".format(path.resolve()) for path in [*outputs.values(), config_path, report_path]),
        ), encoding="utf-8",
    )
    print("Verified {} points; wrote outputs to {}".format(len(xyz), OUT))


if __name__ == "__main__":
    main()
