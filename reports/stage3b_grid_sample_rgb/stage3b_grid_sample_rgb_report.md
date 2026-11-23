# Stage3B — Grid-as-Sample RGB Unsupervised Clustering (24 samples, K=6)

## 输入与网格

- 原始点云和 RGB：`/home/root01/ramanseg/data/processed/roi/ramanseg_1120.ply`。
- 当前 K=6 点级 CSV（用于核对原始点顺序，不用于最终分类）：`/home/root01/ramanseg/reports/agarose_rgb_k6/point_assignments_rgb_k6.csv`。
- 网格边界配置：`/home/root01/ramanseg/reports/stage3_grid4x6/grid_config.json`；沿用 Stage3 当前 4列×6行 XY 边界。
- 有效点数：940；24格点数总和：940；每格点数范围：29–51。
- 样本特征矩阵：24×21；R/G/B 每通道 mean/std/median/min/max/q25/q75。只使用 RGB 统计特征。

## 样本聚类

- StandardScaler 对 24×21 特征矩阵进行标准化；KMeans 参数：`n_clusters=6, n_init=100, random_state=42`。未重跑点级 KMeans，也未使用人工标签。
- 24 个网格样本的 cluster_id：

| Grid | 点数 | Cluster |
|---|---:|---:|
| R1C1 | 35 | 0 |
| R1C2 | 39 | 0 |
| R1C3 | 34 | 0 |
| R1C4 | 51 | 5 |
| R2C1 | 37 | 1 |
| R2C2 | 37 | 1 |
| R2C3 | 39 | 1 |
| R2C4 | 46 | 2 |
| R3C1 | 37 | 1 |
| R3C2 | 40 | 1 |
| R3C3 | 36 | 1 |
| R3C4 | 45 | 1 |
| R4C1 | 41 | 3 |
| R4C2 | 42 | 3 |
| R4C3 | 39 | 1 |
| R4C4 | 43 | 3 |
| R5C1 | 29 | 3 |
| R5C2 | 37 | 3 |
| R5C3 | 37 | 3 |
| R5C4 | 43 | 3 |
| R6C1 | 37 | 2 |
| R6C2 | 38 | 4 |
| R6C3 | 40 | 4 |
| R6C4 | 38 | 2 |

## Cluster 成员

| Cluster | 样本数 | Grid IDs |
|---:|---:|---|
| 0 | 3 | R1C1;R1C2;R1C3 |
| 1 | 8 | R2C1;R2C2;R2C3;R3C1;R3C2;R3C3;R3C4;R4C3 |
| 2 | 3 | R2C4;R6C1;R6C4 |
| 3 | 7 | R4C1;R4C2;R4C4;R5C1;R5C2;R5C3;R5C4 |
| 4 | 2 | R6C2;R6C3 |
| 5 | 1 | R1C4 |

## PCA观察

PCA 对标准化后的24个网格特征样本降至二维，仅用于可视化。前两主成分解释方差为 62.18% 和 20.28%。聚类分组在该二维投影中的分离程度仅描述当前特征空间，不代表浓度标签或化学类别。

## 局限性

- 每个网格作为一个样本，特征是该格点 RGB 分布统计；同一格内部空间结构被汇总掉。
- 网格边界沿用当前 Stage3 4×6 配置；若物理 ROI 边界有偏差，点数与 RGB 统计会随之变化。
- 只有24个样本而聚为6类，单个样本或异常颜色可能影响聚类；cluster_id 无固有语义。
- RGB 统计受光照、反射、曝光、白平衡和观察角度影响；结果不能证明识别了浓度。

## 输出
 
- `/home/root01/ramanseg/reports/stage3b_grid_sample_rgb/grid_rgb_features.csv`
- `/home/root01/ramanseg/reports/stage3b_grid_sample_rgb/sample_cluster_assignments.csv`
- `/home/root01/ramanseg/reports/stage3b_grid_sample_rgb/sample_rgb_pca_k6.png`
- `/home/root01/ramanseg/reports/stage3b_grid_sample_rgb/grid_sample_cluster_map_k6.png`
- `/home/root01/ramanseg/reports/stage3b_grid_sample_rgb/grid_rgb_feature_heatmap.png`
- `/home/root01/ramanseg/reports/stage3b_grid_sample_rgb/cluster_sample_counts.csv`
- `/home/root01/ramanseg/reports/stage3b_grid_sample_rgb/cluster_membership_summary.csv`
- `/home/root01/ramanseg/reports/stage3b_grid_sample_rgb/grid_sample_cluster_overlay.png`
- `/home/root01/ramanseg/reports/stage3b_grid_sample_rgb/run_config.json`
- `/home/root01/ramanseg/reports/stage3b_grid_sample_rgb/stage3b_grid_sample_rgb_report.md`
