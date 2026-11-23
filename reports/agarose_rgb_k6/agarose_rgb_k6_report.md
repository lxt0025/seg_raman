# Agarose RGB-only 无监督六类点云聚类报告

## 基本信息

- 输入 PLY：`/home/root01/ramanseg/data/processed/roi/ramanseg_1120.ply`；原文件未修改。
- 原始点数：`940`；有效点数：`940`；无效点数：`0`。
- RGB 归一化范围：min=`[0.12549019607843137, 0.09411764705882353, 0.10196078431372549]`，max=`[0.5058823529411764, 0.47058823529411764, 0.48627450980392156]`。
- 特征仅为全局 `[R,G,B]`；使用一次 `StandardScaler`；XYZ 未进入 KMeans。
- KMeans：`n_clusters=6`，`n_init=50`，`random_state=42`，`max_iter=500`。
- PCA 只用于二维显示，PC1/PC2 解释方差分别为 `89.587%`、`7.906%`。
- 空间连通分析半径：`0.002302 m`；仅用于解释，没有反馈给 RGB KMeans。
- 当前输入按用户确认不含背景；Cluster 0~5 只是无监督编号，没有自动赋予浓度含义。

## Cluster 组成

| Cluster | 点数 | 比例 | RGB 均值 | RGB 中位数 | 空间质心 XYZ (m) | XYZ 范围 (m) |
|---:|---:|---:|---|---|---|---|
| 0 | 263 | 27.979% | (0.222, 0.151, 0.162) | (0.220, 0.153, 0.161) | (-0.3695, -0.2141, 0.0388) | (0.0346, 0.0539, 0.0143) |
| 1 | 228 | 24.255% | (0.171, 0.117, 0.136) | (0.173, 0.118, 0.137) | (-0.3724, -0.2079, 0.0386) | (0.0342, 0.0521, 0.0118) |
| 2 | 1 | 0.106% | (0.506, 0.471, 0.486) | (0.506, 0.471, 0.486) | (-0.3724, -0.2395, 0.0402) | (0.0000, 0.0000, 0.0000) |
| 3 | 11 | 1.170% | (0.335, 0.276, 0.293) | (0.329, 0.271, 0.294) | (-0.3683, -0.2159, 0.0396) | (0.0325, 0.0538, 0.0038) |
| 4 | 42 | 4.468% | (0.254, 0.179, 0.194) | (0.249, 0.176, 0.188) | (-0.3676, -0.2008, 0.0387) | (0.0347, 0.0541, 0.0063) |
| 5 | 395 | 42.021% | (0.195, 0.138, 0.151) | (0.196, 0.137, 0.153) | (-0.3702, -0.2151, 0.0386) | (0.0328, 0.0536, 0.0129) |

标准化 KMeans 中心、逆变换 RGB 中心、XYZ 最小/最大值等完整数值见 `cluster_statistics_rgb_k6.csv`。

## 空间连通分量

按簇统计的半径图连通分量数量：`{0: 35, 1: 27, 2: 1, 3: 8, 4: 20, 5: 23}`。完整分量点数、质心、包围盒与范围见 `cluster_spatial_components.csv`。

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

- `/home/root01/ramanseg/reports/agarose_rgb_k6/rgb_k6_clustered.ply`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/clusters/cluster_0.ply`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/clusters/cluster_1.ply`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/clusters/cluster_2.ply`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/clusters/cluster_3.ply`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/clusters/cluster_4.ply`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/clusters/cluster_5.ply`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/point_assignments_rgb_k6.csv`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/cluster_statistics_rgb_k6.csv`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/cluster_spatial_components.csv`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/original_rgb.png`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/rgb_k6_clustered.png`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/original_vs_rgb_k6.png`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/rgb_k6_xy.png`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/rgb_k6_xz.png`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/rgb_k6_yz.png`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/rgb_k6_pca.png`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/rgb_k6_pca_coordinates.csv`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/run_config.json`
- `/home/root01/ramanseg/reports/agarose_rgb_k6/agarose_rgb_k6_report.md`
