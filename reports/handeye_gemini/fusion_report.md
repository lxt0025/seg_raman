# RamanSeg Gemini 彩色点云融合报告

## 结论

- 实际融合帧数：`75`；帧号：`[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75]`。
- 排除融合帧：`[]`。Frame 42 已正常参与融合。
- 输入点总数：`39040927`。
- 每帧 2 mm 体素降采样后点数合计：`5428788`。
- 最终输出点数：`865203`。
- XYZ 最小边界 (m)：`[-2.1650766155329535, -3.3930501477603627, -2.3463578181697575]`。
- XYZ 最大边界 (m)：`[2.1354988879922727, 0.09703870984783558, 0.1666669425785111]`。
- XYZ 范围 (m)：`[4.300575503525226, 3.4900888576081983, 2.5130247607482685]`。

## 融合配置与坐标链

- 输入：`frame-*.pointcloud_color_rgb_correct.ply`，单位 m，坐标系为 RGB Camera。
- 变换：`T_cam2base=T_gripper2base@T_cam2gripper`。
- aligned depth 已经 D2C 对齐；没有再次应用 Depth-to-Color 外参。
- `cloud_unit=m`，`voxel_size=0.002 m`，`merge_cleanup_interval=5`，`frame_offset=0`。
- `exclude_frames=[]`；`save_transformed_frames=True`。
- 每一帧转换后点云均保存到 Robot Base 坐标系目录。

## 输出文件（绝对路径）

- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000001.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000002.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000003.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000004.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000005.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000006.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000007.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000008.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000009.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000010.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000011.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000012.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000013.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000014.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000015.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000016.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000017.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000018.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000019.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000020.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000021.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000022.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000023.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000024.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000025.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000026.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000027.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000028.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000029.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000030.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000031.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000032.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000033.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000034.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000035.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000036.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000037.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000038.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000039.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000040.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000041.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000042.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000043.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000044.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000045.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000046.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000047.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000048.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000049.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000050.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000051.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000052.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000053.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000054.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000055.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000056.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000057.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000058.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000059.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000060.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000061.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000062.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000063.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000064.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000065.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000066.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000067.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000068.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000069.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000070.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000071.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000072.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000073.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000074.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/per_frame_base/frame-000075.pointcloud_in_robot_base.ply`
- `/home/root01/ramanseg/data/processed/merged/ramanseg_gemini_merged_robot_base.ply`
- `/home/root01/ramanseg/reports/handeye_gemini/fusion_stats.json`

## 运行日志（绝对路径）

- `/home/root01/ramanseg/reports/handeye_gemini/fusion_run.log`

- 本报告：`/home/root01/ramanseg/reports/handeye_gemini/fusion_report.md`
- 适配融合脚本：`/home/root01/ramanseg/scripts/transform_and_merge_pointclouds.py`
- 原始融合脚本（未覆盖）：`/home/root01/HandEyeCalibration/HandEyeCalibration-using-OpenCV/transform_and_merge_pointclouds.py`
