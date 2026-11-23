# RGB K=6 点云二维叠加可视化报告

- 原始 PLY：`/home/root01/ramanseg/data/processed/roi/ramanseg_1120.ply`
- K=6 聚类 CSV：`/home/root01/ramanseg/reports/agarose_rgb_k6/point_assignments_rgb_k6.csv`
- 绘制点数：`940`（原始有效点与 CSV 行数一致；索引和 XYZ/RGB 对齐）。
- 投影：正交 XY（X 横轴、Y 纵轴）；另输出 XZ、YZ。
- 底图 alpha：`0.38`；覆盖层 alpha：`0.7`。
- 底图点大小：`34`；聚类点大小：`19`。
- 固定簇颜色：Cluster 0: red, Cluster 1: blue, Cluster 2: green, Cluster 3: orange, Cluster 4: purple, Cluster 5: cyan

本图使用已有 K=6 聚类结果，没有重新运行聚类，也没有更改簇编号。原始 RGB 底图有意降低视觉权重，聚类覆盖层用于突出空间结构，便于观察簇的分布。

XY 底图由原始 PLY 中每个有效点的 `(X,Y)` 正交投影直接绘制，并使用该点原始 RGB；为使底图在同位置的聚类点下仍可见，底图点略大且聚类覆盖点采用半透明较小点绘制。

## 输出文件

- `/home/root01/ramanseg/reports/rgb_k6_overlay/rgb_k6_xy_overlay.png`
- `/home/root01/ramanseg/reports/rgb_k6_overlay/rgb_k6_xy_cluster_only.png`
- `/home/root01/ramanseg/reports/rgb_k6_overlay/rgb_k6_xy_base_only.png`
- `/home/root01/ramanseg/reports/rgb_k6_overlay/rgb_k6_xz_overlay.png`
- `/home/root01/ramanseg/reports/rgb_k6_overlay/rgb_k6_yz_overlay.png`
- `/home/root01/ramanseg/reports/rgb_k6_overlay/overlay_run_config.json`
- `/home/root01/ramanseg/reports/rgb_k6_overlay/overlay_report.md`
