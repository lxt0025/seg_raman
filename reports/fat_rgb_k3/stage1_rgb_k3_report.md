# RGB-only 无监督三类点云聚类报告

## 实验设置

- 输入：`/home/root01/ramanseg/data/processed/roi/ramanseg_1119.ply`。
- 原始点数：`7144`；有效点数：`7144`；无效 XYZ/RGB 点数：`0`。
- 输入 RGB 编码：`normalized_[0,1]`；聚类前统一为 `[0,1]`。
- 特征严格为 `[R,G,B]`，使用 `StandardScaler`；XYZ 仅用于显示和导出。
- KMeans：`n_clusters=3`，`n_init=50`，`random_state=42`。
- 聚类显示调色板固定为 Cluster 0=红、Cluster 1=蓝、Cluster 2=绿；这些显示颜色不代表点的原始 RGB 或组织语义。三个独立 Cluster PLY 保留各点原始 RGB，便于在 CloudCompare 中人工判断。
- 未使用人工标签、Raman、HSV、Lab、法向量、几何特征、深度学习或主动背景删除。
- Cluster 0/1/2 只是算法编号，没有自动赋予 Background/Fat/Non-fat 语义。

## 定量结果

| Cluster | 点数 | 比例 | 平均 RGB (8-bit) | 中位 RGB (8-bit) | KMeans 中心 RGB (8-bit) |
|---:|---:|---:|---|---|---|
| 0 | 1646 | 23.040% | (32.98, 144.33, 193.98) | (27.00, 142.00, 196.00) | (32.98, 144.33, 193.98) |
| 1 | 3040 | 42.553% | (197.66, 181.76, 159.54) | (203.00, 185.50, 161.00) | (197.65, 181.75, 159.54) |
| 2 | 2458 | 34.406% | (161.24, 107.37, 85.71) | (169.00, 107.00, 82.00) | (161.24, 107.35, 85.70) |

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

- `/home/root01/ramanseg/reports/fat_rgb_k3/clustered_rgb_k3.ply`
- `/home/root01/ramanseg/reports/fat_rgb_k3/cluster_0.ply`
- `/home/root01/ramanseg/reports/fat_rgb_k3/cluster_1.ply`
- `/home/root01/ramanseg/reports/fat_rgb_k3/cluster_2.ply`
- `/home/root01/ramanseg/reports/fat_rgb_k3/point_assignments_rgb_k3.csv`
- `/home/root01/ramanseg/reports/fat_rgb_k3/cluster_statistics.csv`
- `/home/root01/ramanseg/reports/fat_rgb_k3/original_rgb.png`
- `/home/root01/ramanseg/reports/fat_rgb_k3/clustered_rgb_k3.png`
- `/home/root01/ramanseg/reports/fat_rgb_k3/original_vs_clustered.png`
- `/home/root01/ramanseg/reports/fat_rgb_k3/projection_xy.png`
- `/home/root01/ramanseg/reports/fat_rgb_k3/projection_xz.png`
- `/home/root01/ramanseg/reports/fat_rgb_k3/projection_yz.png`
- `/home/root01/ramanseg/reports/fat_rgb_k3/stage1_rgb_k3_report.md`
