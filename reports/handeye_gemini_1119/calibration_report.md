# RamanSeg 1119 Gemini 手眼标定报告

## 结论

- 输入帧数：`50`；有效标定帧数：`50`。
- 使用的标定帧：`[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50]`。
- 排除的标定帧：`[]`（仅因棋盘格未检出，不影响点云融合）。
- 最终算法：`TSAI`；矩阵方向：`T_cam2gripper`。
- 固定靶标平移残差：平均 `0.156037898 mm`，中位数 `0.133552767 mm`，最大 `0.351516012 mm`。
- 固定靶标旋转残差：平均 `0.027106467 deg`，中位数 `0.025366759 deg`，最大 `0.061280182 deg`。
- MAD 异常帧：`[]`，共 `0` 帧。

## T_cam2gripper

```text
[-0.086148955533, 0.996270348838, 0.004873344473, 0.004868469362]
[-0.996280966495, -0.086139905825, -0.002037749585, 0.017603490253]
[-0.001610360056, -0.005030770341, 0.999986048948, 0.014932719318]
[0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000]
```

## 坐标和输入核验

- 棋盘格：`pattern_size=(11, 8)`，`square_size=0.015 m`，固定靶标。
- RGB 内参和 OpenCV 顺序畸变直接读取本数据集 `camera_info.json`，没有重估内参。
- 机器人位姿为活动 TCP 的 `T_gripper2base`；Tool ID=1，User Frame ID=0；平移 mm→m。
- RPY 采用已验证的 JAKA `rpy_to_rot_matrix` 约定 `Rz(rz)@Ry(ry)@Rx(rx)`。
- `RunRobotWorldHandEye=False`；保留原始 `create_object_points()` 和 `solvePnP` 流程。
- 位姿旋转最大 `|R.T@R-I|=8.882e-16`，det(R) 范围 `[0.999999999999999, 1.000000000000001]`。

## 五种算法

| 算法 | 有效 | 平均平移残差 (mm) | 平均旋转残差 (deg) | 异常帧数 | 选中 |
|---|---:|---:|---:|---:|---:|
| TSAI | 1 | 0.156037898 | 0.027106467 | 0 | 1 |
| PARK | 1 | 0.156406847 | 0.027105696 | 0 | 0 |
| HORAUD | 1 | 0.156407017 | 0.027105604 | 0 | 0 |
| ANDREFF | 1 | 0.169333506 | 0.027097118 | 0 | 0 |
| DANIILIDIS | 1 | 0.156368089 | 0.027131719 | 0 | 0 |

选择规则：在合法刚体结果中取固定靶标平均平移残差最小者，旋转残差用于同值决胜。

## 逐帧残差

| Frame | 平移残差 (mm) | 旋转残差 (deg) | 异常 |
|---:|---:|---:|---:|
| 1 | 0.110318038 | 0.025708379 | 否 |
| 2 | 0.123702581 | 0.043881420 | 否 |
| 3 | 0.138872513 | 0.032075219 | 否 |
| 4 | 0.063792555 | 0.014597360 | 否 |
| 5 | 0.071743993 | 0.017450539 | 否 |
| 6 | 0.114858340 | 0.023047091 | 否 |
| 7 | 0.075070421 | 0.010587375 | 否 |
| 8 | 0.084686651 | 0.016383007 | 否 |
| 9 | 0.105891762 | 0.043381693 | 否 |
| 10 | 0.140060369 | 0.061280182 | 否 |
| 11 | 0.111407473 | 0.036713359 | 否 |
| 12 | 0.314750650 | 0.028244908 | 否 |
| 13 | 0.283432696 | 0.054013275 | 否 |
| 14 | 0.185031123 | 0.011975360 | 否 |
| 15 | 0.183026785 | 0.011886294 | 否 |
| 16 | 0.197024819 | 0.024213520 | 否 |
| 17 | 0.301494135 | 0.016584326 | 否 |
| 18 | 0.249007585 | 0.007791571 | 否 |
| 19 | 0.275060968 | 0.044527066 | 否 |
| 20 | 0.186026036 | 0.004330758 | 否 |
| 21 | 0.050903822 | 0.036177700 | 否 |
| 22 | 0.190138698 | 0.054754163 | 否 |
| 23 | 0.148039377 | 0.025025139 | 否 |
| 24 | 0.125815696 | 0.027719579 | 否 |
| 25 | 0.114628775 | 0.042040392 | 否 |
| 26 | 0.088136281 | 0.017203423 | 否 |
| 27 | 0.140471187 | 0.029243959 | 否 |
| 28 | 0.112798077 | 0.027543829 | 否 |
| 29 | 0.070523487 | 0.049427137 | 否 |
| 30 | 0.097005909 | 0.030434209 | 否 |
| 31 | 0.074176070 | 0.036825081 | 否 |
| 32 | 0.034146552 | 0.020251871 | 否 |
| 33 | 0.284126375 | 0.047637920 | 否 |
| 34 | 0.108498829 | 0.024939493 | 否 |
| 35 | 0.043039032 | 0.022898819 | 否 |
| 36 | 0.092849696 | 0.020013558 | 否 |
| 37 | 0.051863539 | 0.007955472 | 否 |
| 38 | 0.190053509 | 0.035554498 | 否 |
| 39 | 0.145109321 | 0.010035842 | 否 |
| 40 | 0.062921873 | 0.026547309 | 否 |
| 41 | 0.112509317 | 0.006565331 | 否 |
| 42 | 0.174496820 | 0.012312051 | 否 |
| 43 | 0.296055738 | 0.021605562 | 否 |
| 44 | 0.291687996 | 0.022980226 | 否 |
| 45 | 0.265382815 | 0.025930432 | 否 |
| 46 | 0.197800973 | 0.017085079 | 否 |
| 47 | 0.133301826 | 0.028760492 | 否 |
| 48 | 0.304834097 | 0.024108547 | 否 |
| 49 | 0.133803709 | 0.027281147 | 否 |
| 50 | 0.351516012 | 0.047792390 | 否 |

## 标定输出绝对路径

- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000001.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000002.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000003.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000004.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000005.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000006.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000007.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000008.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000009.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000010.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000011.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000012.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000013.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000014.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000015.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000016.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000017.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000018.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000019.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000020.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000021.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000022.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000023.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000024.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000025.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000026.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000027.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000028.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000029.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000030.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000031.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000032.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000033.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000034.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000035.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000036.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000037.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000038.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000039.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000040.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000041.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000042.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000043.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000044.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000045.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000046.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000047.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000048.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000049.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/DetectedCorners/DetectedCorners_000050.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/FixedTargetResiduals_ANDREFF.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/FixedTargetResiduals_DANIILIDIS.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/FixedTargetResiduals_HORAUD.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/FixedTargetResiduals_PARK.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/FixedTargetResiduals_TSAI.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/FixedTargetRotationResidual_ANDREFF.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/FixedTargetRotationResidual_DANIILIDIS.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/FixedTargetRotationResidual_HORAUD.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/FixedTargetRotationResidual_PARK.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/FixedTargetRotationResidual_TSAI.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/FixedTargetTranslationResidual_ANDREFF.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/FixedTargetTranslationResidual_DANIILIDIS.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/FixedTargetTranslationResidual_HORAUD.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/FixedTargetTranslationResidual_PARK.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/FixedTargetTranslationResidual_TSAI.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/T_cam2gripper_ANDREFF.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/T_cam2gripper_DANIILIDIS.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/T_cam2gripper_HORAUD.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/T_cam2gripper_PARK.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/T_cam2gripper_TSAI.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/T_cam2gripper_selected.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/T_gripper2cam_ANDREFF.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/T_gripper2cam_DANIILIDIS.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/T_gripper2cam_HORAUD.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/T_gripper2cam_PARK.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/T_gripper2cam_TSAI.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/FinalTransforms/handeye_method_summary.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/GeminiInputIntrinsics.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000001.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000002.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000003.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000004.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000005.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000006.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000007.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000008.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000009.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000010.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000011.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000012.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000013.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000014.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000015.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000016.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000017.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000018.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000019.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000020.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000021.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000022.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000023.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000024.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000025.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000026.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000027.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000028.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000029.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000030.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000031.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000032.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000033.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000034.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000035.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000036.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000037.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000038.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000039.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000040.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000041.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000042.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000043.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000044.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000045.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000046.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000047.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000048.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000049.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1119/robot_poses/frame-000050.npy`

- 本报告：`/home/root01/ramanseg/reports/handeye_gemini_1119/calibration_report.md`
