# Stage3 — 4×6 Grid-level Unsupervised RGB Classification

## 数据与方法

- 原始点云：`/home/root01/ramanseg/data/processed/roi/ramanseg_1120.ply`
- K=6 点级 CSV：`/home/root01/ramanseg/reports/agarose_rgb_k6/point_assignments_rgb_k6.csv`
- 选择依据：该 CSV 字段为 point_index/XYZ/RGB/cluster_id，940 行，与当前 ROI PLY 的 940 个有效点按原始点序对齐，XYZ 最大误差 5.000e-13；簇 ID 恰为 0–5。Stage2 四样本 K=4 文件不符合点数和类别数。
- 点云/CSV 点数：940；ROI 内参与分格点数：940；ROI 外点数：0。
- K=6 点级分布：{0: 263, 1: 228, 2: 1, 3: 11, 4: 42, 5: 395}。
- 网格范围：X=[-0.388705611, -0.353619099] m，Y=[-0.239811942, -0.185668573] m；X 方向 4 列，Y 方向 6 行，第一行在图像上方。
- 每格尺寸：宽 0.008772 m，高 0.009024 m，宽高比 0.9720；未修改原始坐标。
- 无已保存 KMeans/Scaler；使用点级标签多数投票。投票占比是网格内标签比例，不是校准概率。

## 24 格分类

| 网格 | 点数 | C0 | C1 | C2 | C3 | C4 | C5 | 最终类别 | 主类比例 | 次类 | 次类比例 | 差值 | 状态 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|
| R1C1 | 35 | 11 | 7 | 0 | 1 | 5 | 11 | 0 | 0.314 | 5 | 0.314 | 0.000 | LowConfidence |
| R1C2 | 39 | 9 | 6 | 0 | 1 | 2 | 21 | 5 | 0.538 | 0 | 0.231 | 0.308 | OK |
| R1C3 | 34 | 14 | 3 | 0 | 0 | 3 | 14 | 0 | 0.412 | 5 | 0.412 | 0.000 | LowConfidence |
| R1C4 | 51 | 31 | 0 | 0 | 3 | 17 | 0 | 0 | 0.608 | 4 | 0.333 | 0.275 | OK |
| R2C1 | 37 | 5 | 16 | 0 | 0 | 0 | 16 | 1 | 0.432 | 5 | 0.432 | 0.000 | LowConfidence |
| R2C2 | 37 | 0 | 31 | 0 | 0 | 0 | 6 | 1 | 0.838 | 5 | 0.162 | 0.676 | OK |
| R2C3 | 39 | 7 | 18 | 0 | 0 | 0 | 14 | 1 | 0.462 | 5 | 0.359 | 0.103 | OK |
| R2C4 | 46 | 26 | 2 | 0 | 0 | 3 | 15 | 0 | 0.565 | 5 | 0.326 | 0.239 | OK |
| R3C1 | 37 | 3 | 26 | 0 | 0 | 0 | 8 | 1 | 0.703 | 5 | 0.216 | 0.486 | OK |
| R3C2 | 40 | 3 | 18 | 0 | 0 | 0 | 19 | 5 | 0.475 | 1 | 0.450 | 0.025 | LowConfidence |
| R3C3 | 36 | 1 | 26 | 0 | 0 | 0 | 9 | 1 | 0.722 | 5 | 0.250 | 0.472 | OK |
| R3C4 | 45 | 3 | 18 | 0 | 0 | 1 | 23 | 5 | 0.511 | 1 | 0.400 | 0.111 | OK |
| R4C1 | 41 | 16 | 2 | 0 | 0 | 3 | 20 | 5 | 0.488 | 0 | 0.390 | 0.098 | LowConfidence |
| R4C2 | 42 | 9 | 13 | 0 | 0 | 0 | 20 | 5 | 0.476 | 1 | 0.310 | 0.167 | OK |
| R4C3 | 39 | 2 | 14 | 0 | 0 | 0 | 23 | 5 | 0.590 | 1 | 0.359 | 0.231 | OK |
| R4C4 | 43 | 10 | 7 | 0 | 0 | 0 | 26 | 5 | 0.605 | 0 | 0.233 | 0.372 | OK |
| R5C1 | 29 | 2 | 1 | 0 | 0 | 0 | 26 | 5 | 0.897 | 0 | 0.069 | 0.828 | OK |
| R5C2 | 37 | 8 | 5 | 0 | 0 | 0 | 24 | 5 | 0.649 | 0 | 0.216 | 0.432 | OK |
| R5C3 | 37 | 2 | 7 | 0 | 0 | 0 | 28 | 5 | 0.757 | 1 | 0.189 | 0.568 | OK |
| R5C4 | 43 | 14 | 3 | 0 | 0 | 1 | 25 | 5 | 0.581 | 0 | 0.326 | 0.256 | OK |
| R6C1 | 37 | 30 | 1 | 0 | 0 | 2 | 4 | 0 | 0.811 | 5 | 0.108 | 0.703 | OK |
| R6C2 | 38 | 20 | 1 | 1 | 2 | 3 | 11 | 0 | 0.526 | 5 | 0.289 | 0.237 | OK |
| R6C3 | 40 | 16 | 0 | 0 | 4 | 1 | 19 | 5 | 0.475 | 0 | 0.400 | 0.075 | LowConfidence |
| R6C4 | 38 | 21 | 3 | 0 | 0 | 1 | 13 | 0 | 0.553 | 5 | 0.342 | 0.211 | OK |

## 解释与限制

- LowConfidence 判据：主、次类投票比例差小于 0.10；Insufficient：少于 3 点。LowConfidence 网格：['R1C1', 'R1C3', 'R2C1', 'R3C2', 'R4C1', 'R6C3']；Insufficient 网格：[]。
- 六个 Cluster 在 24 格中的点数见上表及 `grid_4x6_cluster_scores.csv`；这些仍是既有 RGB 无监督簇编号，没有浓度标签。
- ROI 初始边界由当前有效点云 XY 外接范围生成。若矩阵边缘含杂点，可编辑 `grid_config.json` 的 xmin/xmax/ymin/ymax 后重新运行；`grid_4x6_bounds_preview.png` 用于检查网格位置。
- 多数投票受每格点数和原有 K=6 聚类质量影响；比例未校准为概率。点少的网格更不稳定，类别相近时不应强行解释为可靠结果。网格分类只是空间汇总，不构成监督分类。

## 输出

- `/home/root01/ramanseg/reports/stage3_grid4x6/grid_config.json`
- `/home/root01/ramanseg/reports/stage3_grid4x6/grid_4x6_bounds_preview.png`
- `/home/root01/ramanseg/reports/stage3_grid4x6/grid_4x6_labels.csv`
- `/home/root01/ramanseg/reports/stage3_grid4x6/grid_4x6_cluster_scores.csv`
- `/home/root01/ramanseg/reports/stage3_grid4x6/grid_4x6_point_assignments.png`
- `/home/root01/ramanseg/reports/stage3_grid4x6/grid_4x6_dominant_cluster_map.png`
- `/home/root01/ramanseg/reports/stage3_grid4x6/grid_4x6_overlay.png`
- `/home/root01/ramanseg/reports/stage3_grid4x6/grid_4x6_score_heatmap.png`
