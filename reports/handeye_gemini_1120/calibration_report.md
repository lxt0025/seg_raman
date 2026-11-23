# RamanSeg 1120 Gemini 手眼标定报告

## 结论

- 输入帧数：`78`；有效标定帧数：`78`。
- 使用的标定帧：`[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78]`。
- 排除的标定帧：`[]`（仅因棋盘格未检出，不影响点云融合）。
- 最终算法：`HORAUD`；矩阵方向：`T_cam2gripper`。
- 固定靶标平移残差：平均 `0.151359262 mm`，中位数 `0.143295896 mm`，最大 `0.370460461 mm`。
- 固定靶标旋转残差：平均 `0.029054480 deg`，中位数 `0.026363036 deg`，最大 `0.079677056 deg`。
- MAD 异常帧：`[1, 2, 64, 65]`，共 `4` 帧。

## T_cam2gripper

```text
[0.081430521378, 0.996573738351, 0.014486345883, 0.001302871941]
[-0.996596572688, 0.081602132615, -0.011677468004, 0.006986360743]
[-0.012819574661, -0.013486140350, 0.999826876276, 0.015984260594]
[0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000]
```

## 坐标和输入核验

- 棋盘格：`pattern_size=(11, 8)`，`square_size=0.015 m`，固定靶标。
- RGB 内参和 OpenCV 顺序畸变直接读取本数据集 `camera_info.json`，没有重估内参。
- 机器人位姿为活动 TCP 的 `T_gripper2base`；Tool ID=1，User Frame ID=0；平移 mm→m。
- RPY 采用已验证的 JAKA `rpy_to_rot_matrix` 约定 `Rz(rz)@Ry(ry)@Rx(rx)`。
- `RunRobotWorldHandEye=False`；保留原始 `create_object_points()` 和 `solvePnP` 流程。
- 位姿旋转最大 `|R.T@R-I|=6.661e-16`，det(R) 范围 `[0.999999999999999, 1.000000000000001]`。

## 五种算法

| 算法 | 有效 | 平均平移残差 (mm) | 平均旋转残差 (deg) | 异常帧数 | 选中 |
|---|---:|---:|---:|---:|---:|
| TSAI | 1 | 0.152882399 | 0.029246898 | 4 | 0 |
| PARK | 1 | 0.151360639 | 0.029054406 | 4 | 0 |
| HORAUD | 1 | 0.151359262 | 0.029054480 | 4 | 1 |
| ANDREFF | 1 | 0.152880839 | 0.029053598 | 3 | 0 |
| DANIILIDIS | 1 | 0.154638042 | 0.029244476 | 4 | 0 |

选择规则：在合法刚体结果中取固定靶标平均平移残差最小者，旋转残差用于同值决胜。

## 逐帧残差

| Frame | 平移残差 (mm) | 旋转残差 (deg) | 异常 |
|---:|---:|---:|---:|
| 1 | 0.340772898 | 0.049631610 | 是 |
| 2 | 0.370460461 | 0.052744454 | 是 |
| 3 | 0.220174625 | 0.040428751 | 否 |
| 4 | 0.284491793 | 0.043531025 | 否 |
| 5 | 0.164125715 | 0.004644338 | 否 |
| 6 | 0.171462280 | 0.033025266 | 否 |
| 7 | 0.186542145 | 0.024365827 | 否 |
| 8 | 0.203474577 | 0.022596857 | 否 |
| 9 | 0.194461990 | 0.040150519 | 否 |
| 10 | 0.199490235 | 0.032578097 | 否 |
| 11 | 0.245434949 | 0.041490870 | 否 |
| 12 | 0.148267599 | 0.028643917 | 否 |
| 13 | 0.158314471 | 0.015800204 | 否 |
| 14 | 0.153609553 | 0.011274944 | 否 |
| 15 | 0.155601757 | 0.022023242 | 否 |
| 16 | 0.115441700 | 0.034291227 | 否 |
| 17 | 0.103623226 | 0.008739204 | 否 |
| 18 | 0.107136708 | 0.020318946 | 否 |
| 19 | 0.094887609 | 0.031431203 | 否 |
| 20 | 0.085597739 | 0.012770528 | 否 |
| 21 | 0.095308758 | 0.020349570 | 否 |
| 22 | 0.136529615 | 0.015036450 | 否 |
| 23 | 0.077470075 | 0.006119530 | 否 |
| 24 | 0.092903429 | 0.040679505 | 否 |
| 25 | 0.121888073 | 0.016917294 | 否 |
| 26 | 0.151695267 | 0.031131084 | 否 |
| 27 | 0.103128354 | 0.007759103 | 否 |
| 28 | 0.105403471 | 0.027152093 | 否 |
| 29 | 0.134442457 | 0.015761226 | 否 |
| 30 | 0.093999772 | 0.014960537 | 否 |
| 31 | 0.102127792 | 0.019356189 | 否 |
| 32 | 0.059807332 | 0.041611168 | 否 |
| 33 | 0.037679802 | 0.018541709 | 否 |
| 34 | 0.074451551 | 0.012598841 | 否 |
| 35 | 0.041406302 | 0.034312031 | 否 |
| 36 | 0.112256396 | 0.036882329 | 否 |
| 37 | 0.154912611 | 0.023463216 | 否 |
| 38 | 0.168886879 | 0.009811017 | 否 |
| 39 | 0.121106406 | 0.014648568 | 否 |
| 40 | 0.086739582 | 0.023630446 | 否 |
| 41 | 0.084227288 | 0.013725920 | 否 |
| 42 | 0.142032637 | 0.027536893 | 否 |
| 43 | 0.220758426 | 0.056826029 | 否 |
| 44 | 0.129690922 | 0.013433403 | 否 |
| 45 | 0.164668875 | 0.036076198 | 否 |
| 46 | 0.051713088 | 0.022537528 | 否 |
| 47 | 0.059768960 | 0.015211010 | 否 |
| 48 | 0.109095808 | 0.011612794 | 否 |
| 49 | 0.150824829 | 0.025572515 | 否 |
| 50 | 0.124180131 | 0.033287328 | 否 |
| 51 | 0.178139080 | 0.030493384 | 否 |
| 52 | 0.265347115 | 0.020733689 | 否 |
| 53 | 0.164334161 | 0.011877089 | 否 |
| 54 | 0.049318638 | 0.042819516 | 否 |
| 55 | 0.119628336 | 0.020118641 | 否 |
| 56 | 0.051529650 | 0.032032554 | 否 |
| 57 | 0.081653415 | 0.020173359 | 否 |
| 58 | 0.127291025 | 0.034182057 | 否 |
| 59 | 0.156324722 | 0.056042468 | 否 |
| 60 | 0.278502751 | 0.036474857 | 否 |
| 61 | 0.276796119 | 0.055351242 | 否 |
| 62 | 0.174467675 | 0.043214073 | 否 |
| 63 | 0.204524932 | 0.042630691 | 否 |
| 64 | 0.241529073 | 0.079677056 | 是 |
| 65 | 0.239440260 | 0.078811515 | 是 |
| 66 | 0.197854957 | 0.045733793 | 否 |
| 67 | 0.138183395 | 0.040787211 | 否 |
| 68 | 0.138979120 | 0.032657216 | 否 |
| 69 | 0.182086536 | 0.015322079 | 否 |
| 70 | 0.200947591 | 0.030146167 | 否 |
| 71 | 0.224036529 | 0.025322513 | 否 |
| 72 | 0.137725647 | 0.021848146 | 否 |
| 73 | 0.144559156 | 0.053871657 | 否 |
| 74 | 0.181235746 | 0.025573978 | 否 |
| 75 | 0.220518645 | 0.039749850 | 否 |
| 76 | 0.227212112 | 0.007398660 | 否 |
| 77 | 0.075569963 | 0.023284364 | 否 |
| 78 | 0.115809203 | 0.042899062 | 否 |

## 标定输出绝对路径

- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000001.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000002.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000003.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000004.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000005.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000006.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000007.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000008.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000009.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000010.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000011.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000012.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000013.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000014.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000015.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000016.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000017.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000018.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000019.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000020.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000021.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000022.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000023.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000024.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000025.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000026.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000027.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000028.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000029.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000030.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000031.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000032.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000033.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000034.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000035.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000036.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000037.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000038.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000039.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000040.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000041.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000042.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000043.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000044.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000045.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000046.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000047.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000048.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000049.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000050.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000051.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000052.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000053.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000054.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000055.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000056.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000057.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000058.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000059.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000060.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000061.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000062.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000063.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000064.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000065.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000066.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000067.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000068.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000069.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000070.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000071.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000072.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000073.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000074.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000075.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000076.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000077.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/DetectedCorners/DetectedCorners_000078.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/FixedTargetResiduals_ANDREFF.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/FixedTargetResiduals_DANIILIDIS.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/FixedTargetResiduals_HORAUD.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/FixedTargetResiduals_PARK.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/FixedTargetResiduals_TSAI.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/FixedTargetRotationResidual_ANDREFF.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/FixedTargetRotationResidual_DANIILIDIS.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/FixedTargetRotationResidual_HORAUD.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/FixedTargetRotationResidual_PARK.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/FixedTargetRotationResidual_TSAI.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/FixedTargetTranslationResidual_ANDREFF.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/FixedTargetTranslationResidual_DANIILIDIS.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/FixedTargetTranslationResidual_HORAUD.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/FixedTargetTranslationResidual_PARK.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/FixedTargetTranslationResidual_TSAI.png`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/T_cam2gripper_ANDREFF.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/T_cam2gripper_DANIILIDIS.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/T_cam2gripper_HORAUD.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/T_cam2gripper_PARK.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/T_cam2gripper_TSAI.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/T_cam2gripper_selected.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/T_gripper2cam_ANDREFF.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/T_gripper2cam_DANIILIDIS.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/T_gripper2cam_HORAUD.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/T_gripper2cam_PARK.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/T_gripper2cam_TSAI.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/FinalTransforms/handeye_method_summary.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/GeminiInputIntrinsics.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000001.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000002.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000003.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000004.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000005.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000006.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000007.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000008.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000009.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000010.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000011.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000012.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000013.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000014.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000015.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000016.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000017.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000018.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000019.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000020.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000021.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000022.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000023.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000024.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000025.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000026.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000027.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000028.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000029.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000030.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000031.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000032.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000033.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000034.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000035.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000036.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000037.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000038.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000039.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000040.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000041.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000042.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000043.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000044.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000045.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000046.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000047.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000048.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000049.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000050.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000051.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000052.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000053.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000054.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000055.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000056.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000057.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000058.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000059.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000060.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000061.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000062.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000063.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000064.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000065.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000066.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000067.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000068.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000069.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000070.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000071.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000072.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000073.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000074.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000075.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000076.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000077.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini_1120/robot_poses/frame-000078.npy`

- 本报告：`/home/root01/ramanseg/reports/handeye_gemini_1120/calibration_report.md`
