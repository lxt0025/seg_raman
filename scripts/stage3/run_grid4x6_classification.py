#!/usr/bin/env python3
"""Stage 3: square-like 4-column x 6-row vote over existing RGB K=6 labels."""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import open3d as o3d


ROOT = Path(__file__).resolve().parents[2]
PLY = ROOT / "data/processed/roi/ramanseg_1120.ply"
ASSIGNMENTS = ROOT / "reports/agarose_rgb_k6/point_assignments_rgb_k6.csv"
OUT = ROOT / "reports/stage3_grid4x6"
CONFIG = OUT / "grid_config.json"
N_COLS = 4
N_ROWS = 6
N_CLUSTERS = 6
COLORS = ["#ff3333", "#3366ff", "#22aa44", "#ff941a", "#a52bd1", "#06c9d2"]
LOW_MARGIN_THRESHOLD = 0.10
MIN_POINTS = 3


def load_config(x, y):
    if CONFIG.exists():
        config = json.loads(CONFIG.read_text(encoding="utf-8"))
        for key in ("xmin", "xmax", "ymin", "ymax"):
            config[key] = float(config[key])
        return config
    config = {
        "xmin": float(x.min()), "xmax": float(x.max()),
        "ymin": float(y.min()), "ymax": float(y.max()),
        "n_cols": N_COLS, "n_rows": N_ROWS,
        "initial_bounds_source": "current K=6 ROI point-cloud XY extent",
        "note": "Edit xmin/xmax/ymin/ymax to refine matrix ROI; no point coordinates are changed.",
    }
    OUT.mkdir(parents=True, exist_ok=True)
    CONFIG.write_text(json.dumps(config, indent=2), encoding="utf-8")
    return config


def save_preview(xmin, xmax, ymin, ymax, x, y, outpath):
    fig, ax = plt.subplots(figsize=(10, 7), dpi=180)
    ax.scatter(x, y, s=8, c="#666666", alpha=0.45, linewidths=0)
    for xv in np.linspace(xmin, xmax, N_COLS + 1):
        ax.axvline(xv, color="black", lw=0.9)
    for yv in np.linspace(ymin, ymax, N_ROWS + 1):
        ax.axhline(yv, color="black", lw=0.9)
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("X (m)"); ax.set_ylabel("Y (m)")
    ax.set_title("Initial 4 x 6 Square-like Grid Bounds Preview")
    for row in range(N_ROWS):
        for col in range(N_COLS):
            cx = xmin + (col + 0.5) * (xmax - xmin) / N_COLS
            cy = ymax - (row + 0.5) * (ymax - ymin) / N_ROWS
            ax.text(cx, cy, "R{}C{}".format(row + 1, col + 1), ha="center", va="center", fontsize=8)
    fig.tight_layout(); fig.savefig(outpath, bbox_inches="tight"); plt.close(fig)


def write_csv(path, fieldnames, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader(); writer.writerows(rows)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    cloud = o3d.io.read_point_cloud(str(PLY))
    xyz_all = np.asarray(cloud.points, dtype=float)
    rgb_all = np.asarray(cloud.colors, dtype=float)
    valid = np.isfinite(xyz_all).all(axis=1) & np.isfinite(rgb_all).all(axis=1)
    indices_valid = np.flatnonzero(valid)
    xyz = xyz_all[valid]
    with ASSIGNMENTS.open("r", encoding="utf-8", newline="") as stream:
        rows = list(csv.DictReader(stream))
    fields = ["point_index", "x", "y", "z", "r", "g", "b", "cluster_id"]
    if not rows or list(rows[0]) != fields:
        raise ValueError("Unexpected K=6 assignment CSV fields")
    point_indices = np.array([int(r["point_index"]) for r in rows])
    csv_xyz = np.array([[float(r[k]) for k in ("x", "y", "z")] for r in rows])
    labels = np.array([int(r["cluster_id"]) for r in rows])
    if len(rows) != len(xyz) or not np.array_equal(point_indices, indices_valid):
        raise ValueError("PLY and assignment point counts/order differ")
    if not np.allclose(csv_xyz, xyz, rtol=0, atol=1e-8):
        raise ValueError("CSV XYZ does not match original PLY")
    if set(np.unique(labels)) != set(range(N_CLUSTERS)):
        raise ValueError("Expected Cluster IDs 0..5")

    x, y = xyz[:, 0], xyz[:, 1]
    config = load_config(x, y)
    xmin, xmax, ymin, ymax = (config[k] for k in ("xmin", "xmax", "ymin", "ymax"))
    if not xmin < xmax or not ymin < ymax:
        raise ValueError("Invalid grid_config bounds")
    in_roi = (x >= xmin) & (x <= xmax) & (y >= ymin) & (y <= ymax)
    # Use right/top inclusive handling, with exact max boundaries clamped to final bin.
    cols = np.floor((x - xmin) / (xmax - xmin) * N_COLS).astype(int)
    row_from_bottom = np.floor((y - ymin) / (ymax - ymin) * N_ROWS).astype(int)
    cols = np.clip(cols, 0, N_COLS - 1)
    row_from_bottom = np.clip(row_from_bottom, 0, N_ROWS - 1)
    rows_from_top = N_ROWS - 1 - row_from_bottom
    grid_ids = np.array(["R{}C{}".format(r + 1, c + 1) for r, c in zip(rows_from_top, cols)])

    grid_rows, score_rows, grid_data = [], [], {}
    for row in range(N_ROWS):
        for col in range(N_COLS):
            grid_id = "R{}C{}".format(row + 1, col + 1)
            mask = in_roi & (rows_from_top == row) & (cols == col)
            counts = np.bincount(labels[mask], minlength=N_CLUSTERS)
            count = int(mask.sum())
            ratios = counts / count if count else np.zeros(N_CLUSTERS)
            order = np.argsort(-counts, kind="stable")
            dominant = int(order[0]) if count else ""
            second = int(order[1]) if count else ""
            dominant_fraction = float(ratios[order[0]]) if count else 0.0
            second_fraction = float(ratios[order[1]]) if count else 0.0
            margin = dominant_fraction - second_fraction
            status = "Insufficient" if count < MIN_POINTS else ("LowConfidence" if margin < LOW_MARGIN_THRESHOLD else "OK")
            bx0 = xmin + col * (xmax - xmin) / N_COLS
            bx1 = xmin + (col + 1) * (xmax - xmin) / N_COLS
            by1 = ymax - row * (ymax - ymin) / N_ROWS
            by0 = ymax - (row + 1) * (ymax - ymin) / N_ROWS
            item = {
                "grid_id": grid_id, "row": row + 1, "col": col + 1,
                "xmin": bx0, "xmax": bx1, "ymin": by0, "ymax": by1,
                "point_count": count,
            }
            for k in range(N_CLUSTERS):
                item["cluster_{}_count".format(k)] = int(counts[k])
            for k in range(N_CLUSTERS):
                item["cluster_{}_ratio".format(k)] = float(ratios[k])
            item.update({
                "dominant_cluster": dominant, "dominant_fraction": dominant_fraction,
                "second_cluster": second, "second_fraction": second_fraction,
                "dominance_margin": margin, "status": status,
            })
            grid_rows.append(item)
            score_rows.append({"grid_id": grid_id, **{"cluster_{}".format(k): float(ratios[k]) for k in range(N_CLUSTERS)}})
            grid_data[grid_id] = item

    columns = ["grid_id", "row", "col", "xmin", "xmax", "ymin", "ymax", "point_count"]
    columns += ["cluster_{}_count".format(k) for k in range(6)]
    columns += ["cluster_{}_ratio".format(k) for k in range(6)]
    columns += ["dominant_cluster", "dominant_fraction", "second_cluster", "second_fraction", "dominance_margin", "status"]
    write_csv(OUT / "grid_4x6_labels.csv", columns, grid_rows)
    write_csv(OUT / "grid_4x6_cluster_scores.csv", ["grid_id"] + ["cluster_{}".format(k) for k in range(6)], score_rows)
    save_preview(xmin, xmax, ymin, ymax, x, y, OUT / "grid_4x6_bounds_preview.png")

    # 1. Original point assignments with grid and IDs.
    fig, ax = plt.subplots(figsize=(12, 8), dpi=200)
    for k in range(6):
        m = labels == k
        ax.scatter(x[m], y[m], s=10, c=COLORS[k], label="Cluster {}".format(k), linewidths=0)
    for xv in np.linspace(xmin, xmax, N_COLS + 1): ax.axvline(xv, color="#333333", lw=0.8)
    for yv in np.linspace(ymin, ymax, N_ROWS + 1): ax.axhline(yv, color="#333333", lw=0.8)
    for r in range(N_ROWS):
        for c in range(N_COLS):
            cx = xmin + (c + 0.5) * (xmax - xmin) / N_COLS
            cy = ymax - (r + 0.5) * (ymax - ymin) / N_ROWS
            ax.text(cx, cy, "R{}C{}".format(r + 1, c + 1), ha="center", va="center", fontsize=7,
                    bbox={"facecolor": "white", "alpha": 0.62, "edgecolor": "none", "pad": 1})
    ax.set(xlim=(xmin, xmax), ylim=(ymin, ymax), xlabel="X (m)", ylabel="Y (m)", title="Original XY Point Assignments with 4 x 6 Grid")
    ax.set_aspect("equal", adjustable="box"); ax.legend(ncol=6, loc="upper center", bbox_to_anchor=(0.5, -0.08))
    fig.tight_layout(); fig.savefig(OUT / "grid_4x6_point_assignments.png", bbox_inches="tight"); plt.close(fig)

    # 2. Dominant cluster map, with distinct hatches/edges for uncertain/empty cells.
    fig, ax = plt.subplots(figsize=(12, 8), dpi=200)
    for item in grid_rows:
        x0, x1, y0, y1 = item["xmin"], item["xmax"], item["ymin"], item["ymax"]
        cluster = item["dominant_cluster"]
        face = COLORS[cluster] if cluster != "" else "#eeeeee"
        status = item["status"]
        rect = Rectangle((x0, y0), x1-x0, y1-y0, facecolor=face, alpha=0.72,
                         edgecolor="#222222" if status == "OK" else ("#d62728" if status == "LowConfidence" else "#777777"),
                         linewidth=2.0 if status != "OK" else 0.9,
                         hatch="xx" if status == "Insufficient" else None)
        ax.add_patch(rect)
        label = "{}\nC{} {:.2f}".format(item["grid_id"], cluster if cluster != "" else "-", item["dominant_fraction"])
        ax.text((x0+x1)/2, (y0+y1)/2, label, ha="center", va="center", fontsize=9, weight="bold")
    ax.set(xlim=(xmin, xmax), ylim=(ymin, ymax), xlabel="X (m)", ylabel="Y (m)", title="4 x 6 Grid Dominant Cluster Map")
    ax.set_aspect("equal", adjustable="box"); fig.tight_layout(); fig.savefig(OUT / "grid_4x6_dominant_cluster_map.png", bbox_inches="tight"); plt.close(fig)

    # 3. Transparent image only: each cell filled, labels centered, no axes/margins.
    dpi = 240
    width_in = 7.5
    height_in = width_in * (ymax - ymin) / (xmax - xmin)
    fig = plt.figure(figsize=(width_in, height_in), dpi=dpi, frameon=False)
    ax = fig.add_axes([0, 0, 1, 1])
    ax.set_xlim(xmin, xmax); ax.set_ylim(ymin, ymax); ax.set_aspect("auto")
    ax.set_axis_off(); ax.patch.set_alpha(0); fig.patch.set_alpha(0)
    for item in grid_rows:
        x0, x1, y0, y1 = item["xmin"], item["xmax"], item["ymin"], item["ymax"]
        cluster = item["dominant_cluster"]
        if cluster != "":
            ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,facecolor=COLORS[cluster],alpha=0.30,edgecolor="none"))
            ax.text((x0+x1)/2,(y0+y1)/2,str(cluster),ha="center",va="center",fontsize=14,color="black")
        ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,facecolor="none",edgecolor="black",linewidth=0.75))
    fig.savefig(OUT / "grid_4x6_overlay.png", dpi=dpi, transparent=True, pad_inches=0)
    plt.close(fig)

    # 4. 24 x 6 vote score heatmap.
    matrix = np.array([[r["cluster_{}".format(k)] for k in range(6)] for r in score_rows])
    fig, ax = plt.subplots(figsize=(10, 9), dpi=200)
    image = ax.imshow(matrix, cmap="viridis", vmin=0, vmax=1, aspect="auto")
    ax.set_xticks(range(6), ["Cluster {}".format(k) for k in range(6)])
    ax.set_yticks(range(24), [r["grid_id"] for r in score_rows])
    for i in range(24):
        for j in range(6): ax.text(j, i, "{:.2f}".format(matrix[i,j]), ha="center", va="center", color="white" if matrix[i,j] > 0.52 else "black", fontsize=7)
    fig.colorbar(image, ax=ax, label="Vote score (within-grid fraction)")
    ax.set_title("4 x 6 Grid Cluster Vote Scores")
    fig.tight_layout(); fig.savefig(OUT / "grid_4x6_score_heatmap.png", bbox_inches="tight"); plt.close(fig)

    # Update config with explicit processing record, keeping bounds editable.
    config.update({"n_cols": N_COLS, "n_rows": N_ROWS, "input_ply": str(PLY.resolve()),
                   "assignment_csv": str(ASSIGNMENTS.resolve()), "point_count": len(xyz),
                   "points_in_grid_range": int(in_roi.sum()), "unassigned_outside_grid": int((~in_roi).sum()),
                   "low_margin_threshold": LOW_MARGIN_THRESHOLD, "minimum_points": MIN_POINTS,
                   "model_and_scaler_saved": False, "voting": "point-label majority vote; no distance confidence"})
    CONFIG.write_text(json.dumps(config, indent=2), encoding="utf-8")

    cell_width = (xmax - xmin) / N_COLS
    cell_height = (ymax - ymin) / N_ROWS
    report_lines = [
        "# Stage3 — 4×6 Grid-level Unsupervised RGB Classification", "",
        "## 数据与方法", "",
        "- 原始点云：`{}`".format(PLY.resolve()),
        "- K=6 点级 CSV：`{}`".format(ASSIGNMENTS.resolve()),
        "- 选择依据：该 CSV 字段为 point_index/XYZ/RGB/cluster_id，940 行，与当前 ROI PLY 的 940 个有效点按原始点序对齐，XYZ 最大误差 {:.3e}；簇 ID 恰为 0–5。Stage2 四样本 K=4 文件不符合点数和类别数。".format(float(np.max(np.abs(csv_xyz-xyz)))),
        "- 点云/CSV 点数：{}；ROI 内参与分格点数：{}；ROI 外点数：{}。".format(len(xyz),int(in_roi.sum()),int((~in_roi).sum())),
        "- K=6 点级分布：{}。".format({int(k):int(v) for k,v in zip(*np.unique(labels,return_counts=True))}),
        "- 网格范围：X=[{:.9f}, {:.9f}] m，Y=[{:.9f}, {:.9f}] m；X 方向 4 列，Y 方向 6 行，第一行在图像上方。".format(xmin,xmax,ymin,ymax),
        "- 每格尺寸：宽 {:.6f} m，高 {:.6f} m，宽高比 {:.4f}；未修改原始坐标。".format(cell_width,cell_height,cell_width/cell_height),
        "- 无已保存 KMeans/Scaler；使用点级标签多数投票。投票占比是网格内标签比例，不是校准概率。", "",
        "## 24 格分类", "",
        "| 网格 | 点数 | C0 | C1 | C2 | C3 | C4 | C5 | 最终类别 | 主类比例 | 次类 | 次类比例 | 差值 | 状态 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for r in grid_rows:
        report_lines.append("| {grid_id} | {point_count} | {cluster_0_count} | {cluster_1_count} | {cluster_2_count} | {cluster_3_count} | {cluster_4_count} | {cluster_5_count} | {dominant_cluster} | {dominant_fraction:.3f} | {second_cluster} | {second_fraction:.3f} | {dominance_margin:.3f} | {status} |".format(**r))
    low = [r["grid_id"] for r in grid_rows if r["status"] == "LowConfidence"]
    insufficient = [r["grid_id"] for r in grid_rows if r["status"] == "Insufficient"]
    report_lines += ["", "## 解释与限制", "",
        "- LowConfidence 判据：主、次类投票比例差小于 {:.2f}；Insufficient：少于 {} 点。LowConfidence 网格：{}；Insufficient 网格：{}。".format(LOW_MARGIN_THRESHOLD,MIN_POINTS,low,insufficient),
        "- 六个 Cluster 在 24 格中的点数见上表及 `grid_4x6_cluster_scores.csv`；这些仍是既有 RGB 无监督簇编号，没有浓度标签。",
        "- ROI 初始边界由当前有效点云 XY 外接范围生成。若矩阵边缘含杂点，可编辑 `grid_config.json` 的 xmin/xmax/ymin/ymax 后重新运行；`grid_4x6_bounds_preview.png` 用于检查网格位置。",
        "- 多数投票受每格点数和原有 K=6 聚类质量影响；比例未校准为概率。点少的网格更不稳定，类别相近时不应强行解释为可靠结果。网格分类只是空间汇总，不构成监督分类。", "",
        "## 输出", ""]
    outfiles = ["grid_config.json","grid_4x6_bounds_preview.png","grid_4x6_labels.csv","grid_4x6_cluster_scores.csv","grid_4x6_point_assignments.png","grid_4x6_dominant_cluster_map.png","grid_4x6_overlay.png","grid_4x6_score_heatmap.png"]
    report_lines.extend("- `{}`".format((OUT/f).resolve()) for f in outfiles)
    (OUT/"stage3_grid4x6_report.md").write_text("\n".join(report_lines)+"\n",encoding="utf-8")

    # Machine checks: every ROI point assigned once, all cell points accounted for.
    if int(in_roi.sum()) != sum(r["point_count"] for r in grid_rows):
        raise AssertionError("Grid point total mismatch")
    if any(r["dominant_cluster"] != "" and r["dominant_cluster"] not in range(6) for r in grid_rows):
        raise AssertionError("Invalid dominant cluster")
    print("Points:",len(xyz),"in-grid:",int(in_roi.sum()),"outside:",int((~in_roi).sum()))
    print("Grid point counts:",[r["point_count"] for r in grid_rows])
    print("Grid classifications:",[(r["grid_id"],r["dominant_cluster"],round(r["dominant_fraction"],3),r["status"]) for r in grid_rows])
    print("LowConfidence:",low,"Insufficient:",insufficient)


if __name__ == "__main__":
    main()
