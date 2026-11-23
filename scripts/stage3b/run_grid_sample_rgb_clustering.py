#!/usr/bin/env python3
"""Stage3B: RGB-statistics KMeans across the 24 existing 4x6 grid samples."""

import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np
import open3d as o3d
from sklearn.cluster import KMeans
from sklearn.decomposition import PCA
from sklearn.preprocessing import StandardScaler


ROOT = Path(__file__).resolve().parents[2]
PLY = ROOT / "data/processed/roi/ramanseg_1120.ply"
POINT_CSV = ROOT / "reports/agarose_rgb_k6/point_assignments_rgb_k6.csv"
GRID_CONFIG = ROOT / "reports/stage3_grid4x6/grid_config.json"
OUT = ROOT / "reports/stage3b_grid_sample_rgb"
N_COLS, N_ROWS, K = 4, 6, 6
PALETTE = ["#ff3333", "#3366ff", "#22aa44", "#ff941a", "#a52bd1", "#06c9d2"]
CHANNELS = ("R", "G", "B")
STATISTICS = ("mean", "std", "median", "min", "max", "q25", "q75")


def write_csv(path, fields, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    cloud = o3d.io.read_point_cloud(str(PLY))
    xyz_all = np.asarray(cloud.points, dtype=np.float64)
    rgb_all = np.asarray(cloud.colors, dtype=np.float64)
    if not cloud.has_colors() or rgb_all.shape != xyz_all.shape:
        raise ValueError("ROI PLY must have XYZ and one RGB per point")
    valid = np.isfinite(xyz_all).all(axis=1) & np.isfinite(rgb_all).all(axis=1)
    valid_indices = np.flatnonzero(valid)
    xyz = xyz_all[valid]
    rgb = np.clip(rgb_all[valid], 0.0, 1.0)

    # Validate the current point-level K=6 source, while using original PLY RGB
    # values directly for the per-grid features.
    with POINT_CSV.open("r", encoding="utf-8", newline="") as stream:
        point_rows = list(csv.DictReader(stream))
    expected_fields = ["point_index", "x", "y", "z", "r", "g", "b", "cluster_id"]
    if not point_rows or list(point_rows[0]) != expected_fields:
        raise ValueError("Unexpected existing K=6 point CSV schema")
    point_ids = np.array([int(r["point_index"]) for r in point_rows])
    csv_xyz = np.array([[float(r[k]) for k in ("x", "y", "z")] for r in point_rows])
    if len(point_rows) != len(xyz) or not np.array_equal(point_ids, valid_indices):
        raise ValueError("Point CSV does not match current ROI PLY point order/count")
    if not np.allclose(csv_xyz, xyz, rtol=0, atol=1e-8):
        raise ValueError("Point CSV XYZ does not match original ROI PLY")

    grid_config = json.loads(GRID_CONFIG.read_text(encoding="utf-8"))
    if grid_config.get("n_cols") != N_COLS or grid_config.get("n_rows") != N_ROWS:
        raise ValueError("Saved grid config is not the current 4-column x 6-row layout")
    xmin, xmax, ymin, ymax = [float(grid_config[k]) for k in ("xmin", "xmax", "ymin", "ymax")]
    x, y = xyz[:, 0], xyz[:, 1]
    within = (x >= xmin) & (x <= xmax) & (y >= ymin) & (y <= ymax)
    if not np.all(within):
        raise ValueError("Some valid points fall outside configured 24-grid ROI")
    col = np.floor((x-xmin)/(xmax-xmin)*N_COLS).astype(int)
    row_bottom = np.floor((y-ymin)/(ymax-ymin)*N_ROWS).astype(int)
    col = np.clip(col, 0, N_COLS-1)
    row_bottom = np.clip(row_bottom, 0, N_ROWS-1)
    row_top = N_ROWS - 1 - row_bottom
    grid_index = row_top*N_COLS + col
    if len(grid_index) != len(xyz) or np.any((grid_index < 0) | (grid_index >= 24)):
        raise AssertionError("Each point must be assigned to exactly one grid")

    feature_fields = ["grid_id", "row", "col", "point_count"]
    feature_fields += ["{}_{}".format(ch, stat) for ch in CHANNELS for stat in STATISTICS]
    feature_rows, feature_matrix, sample_grid = [], [], []
    for r in range(N_ROWS):
        for c in range(N_COLS):
            idx = np.flatnonzero((row_top == r) & (col == c))
            if len(idx) == 0:
                raise ValueError("Empty grid sample R{}C{} cannot be clustered".format(r+1,c+1))
            sample_rgb = rgb[idx]
            row_data = {
                "grid_id": "R{}C{}".format(r+1,c+1),
                "row": r+1, "col": c+1, "point_count": len(idx),
            }
            vector = []
            for channel_idx, channel in enumerate(CHANNELS):
                values = sample_rgb[:, channel_idx]
                stats = [
                    np.mean(values), np.std(values, ddof=0), np.median(values),
                    np.min(values), np.max(values), np.percentile(values,25),
                    np.percentile(values,75),
                ]
                for stat_name, stat_value in zip(STATISTICS, stats):
                    row_data["{}_{}".format(channel, stat_name)] = float(stat_value)
                    vector.append(float(stat_value))
            feature_rows.append(row_data)
            feature_matrix.append(vector)
            sample_grid.append((r,c))
    feature_matrix = np.asarray(feature_matrix, dtype=np.float64)
    if feature_matrix.shape != (24,21) or len(feature_rows) != 24:
        raise AssertionError("Expected exactly 24 samples by 21 RGB statistics")
    write_csv(OUT/"grid_rgb_features.csv", feature_fields, feature_rows)

    scaler = StandardScaler()
    standardized = scaler.fit_transform(feature_matrix)
    model = KMeans(n_clusters=K, n_init=100, random_state=42)
    cluster_ids = model.fit_predict(standardized).astype(int)
    if set(cluster_ids.tolist()) - set(range(6)):
        raise AssertionError("KMeans cluster IDs must be in 0..5")

    assignments = []
    for sample, cid in zip(feature_rows, cluster_ids):
        assignments.append({k: sample[k] for k in ("grid_id","row","col","point_count")} | {"cluster_id": int(cid)})
    write_csv(OUT/"sample_cluster_assignments.csv", ["grid_id","row","col","point_count","cluster_id"], assignments)

    # Sample-level PCA, for display only.
    pca = PCA(n_components=2)
    pc = pca.fit_transform(standardized)
    fig, ax = plt.subplots(figsize=(10,8),dpi=220)
    for cid in range(6):
        mask=cluster_ids==cid
        ax.scatter(pc[mask,0],pc[mask,1],s=70,c=PALETTE[cid],label="Cluster {}".format(cid),edgecolors="white",linewidths=0.6)
        for i in np.flatnonzero(mask): ax.annotate(feature_rows[i]["grid_id"],(pc[i,0],pc[i,1]),xytext=(5,4),textcoords="offset points",fontsize=8)
    ev=pca.explained_variance_ratio_*100
    ax.set_xlabel("PC1 ({:.2f}% explained variance)".format(ev[0])); ax.set_ylabel("PC2 ({:.2f}% explained variance)".format(ev[1]))
    ax.set_title("24 Grid Samples - RGB Feature PCA (K=6)"); ax.legend(ncol=3); ax.grid(alpha=.2)
    fig.tight_layout(); fig.savefig(OUT/"sample_rgb_pca_k6.png",bbox_inches="tight"); plt.close(fig)

    # Per-cell cluster map and transparent overlay share exact configured bounds.
    fig,ax=plt.subplots(figsize=(8,11),dpi=200)
    for sample,cid in zip(feature_rows,cluster_ids):
        r,c=sample["row"]-1,sample["col"]-1
        x0=xmin+c*(xmax-xmin)/N_COLS; x1=xmin+(c+1)*(xmax-xmin)/N_COLS
        y1=ymax-r*(ymax-ymin)/N_ROWS; y0=ymax-(r+1)*(ymax-ymin)/N_ROWS
        ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,facecolor=PALETTE[cid],edgecolor="black",linewidth=1.1,alpha=.82))
        ax.text((x0+x1)/2,(y0+y1)/2,"{}\nC{}".format(sample["grid_id"],cid),ha="center",va="center",fontsize=9,weight="bold")
    ax.set(xlim=(xmin,xmax),ylim=(ymin,ymax),xlabel="X (m)",ylabel="Y (m)",title="24 Grid Samples - RGB Feature Clustering (K=6)")
    ax.set_aspect("equal",adjustable="box"); fig.tight_layout(); fig.savefig(OUT/"grid_sample_cluster_map_k6.png",bbox_inches="tight"); plt.close(fig)

    # Heatmap of standardized sample features; sort by assigned cluster then grid order.
    order=sorted(range(24),key=lambda i:(cluster_ids[i],feature_rows[i]["row"],feature_rows[i]["col"]))
    fig,ax=plt.subplots(figsize=(17,10),dpi=200)
    im=ax.imshow(standardized[order],aspect="auto",cmap="coolwarm",vmin=-2.5,vmax=2.5)
    ax.set_yticks(range(24),[feature_rows[i]["grid_id"] for i in order])
    ax.set_xticks(range(21),["{}_{}".format(ch,st) for ch in CHANNELS for st in STATISTICS],rotation=55,ha="right")
    ax.set_title("Standardized RGB Statistics by Grid Sample (sorted by K=6 cluster)")
    fig.colorbar(im,ax=ax,label="Standardized feature value"); fig.tight_layout(); fig.savefig(OUT/"grid_rgb_feature_heatmap.png",bbox_inches="tight"); plt.close(fig)

    counts=np.bincount(cluster_ids,minlength=6)
    count_rows=[{"cluster_id":i,"sample_count":int(counts[i])} for i in range(6)]
    write_csv(OUT/"cluster_sample_counts.csv",["cluster_id","sample_count"],count_rows)
    members=[]
    for cid in range(6):
        ids=[feature_rows[i]["grid_id"] for i in range(24) if cluster_ids[i]==cid]
        members.append({"cluster_id":cid,"member_count":len(ids),"member_grid_ids":";".join(ids)})
    write_csv(OUT/"cluster_membership_summary.csv",["cluster_id","member_count","member_grid_ids"],members)

    # Transparent cell overlay for aligning by the same XY bounds.
    dpi=240; width_in=7.5; height_in=width_in*(ymax-ymin)/(xmax-xmin)
    fig=plt.figure(figsize=(width_in,height_in),dpi=dpi,frameon=False)
    ax=fig.add_axes([0,0,1,1]); ax.set_xlim(xmin,xmax); ax.set_ylim(ymin,ymax); ax.set_axis_off(); ax.set_aspect("auto")
    ax.patch.set_alpha(0); fig.patch.set_alpha(0)
    for sample,cid in zip(feature_rows,cluster_ids):
        r,c=sample["row"]-1,sample["col"]-1
        x0=xmin+c*(xmax-xmin)/N_COLS; x1=xmin+(c+1)*(xmax-xmin)/N_COLS
        y1=ymax-r*(ymax-ymin)/N_ROWS; y0=ymax-(r+1)*(ymax-ymin)/N_ROWS
        ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,facecolor=PALETTE[cid],alpha=.30,edgecolor="none"))
        ax.text((x0+x1)/2,(y0+y1)/2,str(cid),ha="center",va="center",fontsize=14,color="black")
        ax.add_patch(Rectangle((x0,y0),x1-x0,y1-y0,facecolor="none",edgecolor="black",linewidth=.75))
    fig.savefig(OUT/"grid_sample_cluster_overlay.png",dpi=dpi,transparent=True,pad_inches=0); plt.close(fig)

    config={"original_ply":str(PLY.resolve()),"point_level_k6_csv_for_validation":str(POINT_CSV.resolve()),"grid_config":str(GRID_CONFIG.resolve()),
            "grid_columns":N_COLS,"grid_rows":N_ROWS,"sample_count":24,"features_per_sample":21,"feature_channels":list(CHANNELS),"statistics":list(STATISTICS),
            "kmeans":{"n_clusters":6,"n_init":100,"random_state":42},"scaler":"StandardScaler fitted across 24 grid samples","pca_visualization_only":True,
            "cluster_palette":PALETTE,"grid_bounds_m":{"xmin":xmin,"xmax":xmax,"ymin":ymin,"ymax":ymax},"point_count":len(xyz),"point_csv_xyz_max_abs_error":float(np.max(np.abs(csv_xyz-xyz)))}
    (OUT/"run_config.json").write_text(json.dumps(config,indent=2),encoding="utf-8")

    sizes=[r["point_count"] for r in feature_rows]
    lines=["# Stage3B — Grid-as-Sample RGB Unsupervised Clustering (24 samples, K=6)","",
           "## 输入与网格", "",
           "- 原始点云和 RGB：`{}`。".format(PLY.resolve()),
           "- 当前 K=6 点级 CSV（用于核对原始点顺序，不用于最终分类）：`{}`。".format(POINT_CSV.resolve()),
           "- 网格边界配置：`{}`；沿用 Stage3 当前 4列×6行 XY 边界。".format(GRID_CONFIG.resolve()),
           "- 有效点数：{}；24格点数总和：{}；每格点数范围：{}–{}。".format(len(xyz),sum(sizes),min(sizes),max(sizes)),
           "- 样本特征矩阵：24×21；R/G/B 每通道 mean/std/median/min/max/q25/q75。只使用 RGB 统计特征。", "",
           "## 样本聚类", "",
           "- StandardScaler 对 24×21 特征矩阵进行标准化；KMeans 参数：`n_clusters=6, n_init=100, random_state=42`。未重跑点级 KMeans，也未使用人工标签。",
           "- 24 个网格样本的 cluster_id：", "",
           "| Grid | 点数 | Cluster |", "|---|---:|---:|"]
    for a in assignments: lines.append("| {} | {} | {} |".format(a["grid_id"],a["point_count"],a["cluster_id"]))
    lines += ["","## Cluster 成员","","| Cluster | 样本数 | Grid IDs |","|---:|---:|---|"]
    for m in members: lines.append("| {} | {} | {} |".format(m["cluster_id"],m["member_count"],m["member_grid_ids"] or "—"))
    lines += ["","## PCA观察","","PCA 对标准化后的24个网格特征样本降至二维，仅用于可视化。前两主成分解释方差为 {:.2f}% 和 {:.2f}%。聚类分组在该二维投影中的分离程度仅描述当前特征空间，不代表浓度标签或化学类别。".format(ev[0],ev[1]),"",
              "## 局限性","","- 每个网格作为一个样本，特征是该格点 RGB 分布统计；同一格内部空间结构被汇总掉。",
              "- 网格边界沿用当前 Stage3 4×6 配置；若物理 ROI 边界有偏差，点数与 RGB 统计会随之变化。",
              "- 只有24个样本而聚为6类，单个样本或异常颜色可能影响聚类；cluster_id 无固有语义。",
              "- RGB 统计受光照、反射、曝光、白平衡和观察角度影响；结果不能证明识别了浓度。","","## 输出"," "]
    output_files=["grid_rgb_features.csv","sample_cluster_assignments.csv","sample_rgb_pca_k6.png","grid_sample_cluster_map_k6.png","grid_rgb_feature_heatmap.png","cluster_sample_counts.csv","cluster_membership_summary.csv","grid_sample_cluster_overlay.png","run_config.json","stage3b_grid_sample_rgb_report.md"]
    lines.extend("- `{}`".format((OUT/name).resolve()) for name in output_files)
    (OUT/"stage3b_grid_sample_rgb_report.md").write_text("\n".join(lines)+"\n",encoding="utf-8")

    # Required consistency checks.
    if len(feature_rows)!=24 or sum(sizes)!=len(xyz) or len(set(grid_index.tolist()))>24:
        raise AssertionError("Sample/point assignment consistency check failed")
    if any(len(np.flatnonzero(grid_index==i))!=feature_rows[i]["point_count"] for i in range(24)):
        raise AssertionError("Grid point counts mismatch")
    print("Points",len(xyz),"samples",len(feature_rows),"feature shape",feature_matrix.shape,"point count range",min(sizes),max(sizes))
    print("Cluster sample counts",counts.tolist())
    print("Assignments",[(a["grid_id"],a["cluster_id"]) for a in assignments])
    print("Output",OUT)


if __name__=="__main__":
    main()
