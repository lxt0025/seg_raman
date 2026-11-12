# Gemini 336L 手眼标定报告

## 结论

- 最终算法：`TSAI`。
- 最终矩阵方向：`T_cam2gripper`（RGB Camera → 采集期间活动 TCP / Gripper）。
- 标定输入：74 帧；棋盘格固定；`pattern_size=(11, 8)`；`square_size=0.015 m`。
- 标定帧：`[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75]`。
- 排除的标定帧：`[42]`。Frame 42 仅因棋盘格检测失败而从标定排除。
- 平均固定靶标平移残差：`0.223042846 mm`；中位数 `0.200114721 mm`；最大值 `0.495671243 mm`。
- 平均固定靶标旋转残差：`0.038995623 deg`；中位数 `0.036408893 deg`；最大值 `0.106465434 deg`。
- MAD 异常帧：`[59]`；共 `1` 帧。

## T_cam2gripper

```text
[-0.098996480860, 0.995079938440, 0.003951315250, 0.005350622974]
[-0.995084388274, -0.098984989840, -0.003005328680, 0.018991464650]
[-0.002599421378, -0.004229409081, 0.999987677478, 0.015297070400]
[0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000]
```

## 输入与坐标约定核验

- Gemini RGB 内参直接读取 `data/raw/camera_info.json`；未调用 `cv2.calibrateCamera`，未读取旧 `IntrinsicMatrix.npz`。
- RGB 分辨率为 1280×720；畸变顺序使用 `color_camera.distortion.opencv_order`。
- 机器人位置读取 `robot.tcp_pose_raw=[x,y,z,rx,ry,rz]`；平移由 mm 转为 m；RPY 保持 rad。
- Tool ID=1、User Frame ID=0；Gripper Frame 就是采集期间活动 TCP；未转换到法兰，也未使用未经 SDK 验证的平板 Tool Offset。
- RPY 使用 JAKA SDK `rpy_to_rot_matrix` 约定：`R=Rz(rz)@Ry(ry)@Rx(rx)`。用 JAKA SDK 手册参考样例复核的最大元素误差为 `4.779e-07`。
- 75 个旋转矩阵最大 `|R.T@R-I|` 为 `1.110e-15`；det(R) 范围为 `[0.999999999999999, 1.000000000000001]`。
- 机器人位姿方向：`T_gripper2base`；相机位姿由原程序 `solvePnP` 得到 `T_target2cam`。
- 手眼固定靶标链：`T_target2base=T_gripper2base@T_cam2gripper@T_target2cam`。
- `RunRobotWorldHandEye=False`。

JAKA SDK 约定依据：[JAKA Python SDK 的 `rpy_to_rot_matrix`](https://www.jaka.com/docs/documents/SDK/en/python.html)；本机当前未安装可导入的 `jkrc.so/libjakaAPI.so`，因此使用该 SDK 接口的已验证等价矩阵表达式，并以官方数值样例作启动自检。

## 五种算法结果

| 算法 | 有效 | 平均平移残差 (mm) | 平均旋转残差 (deg) | 异常帧数 | 选中 |
|---|---:|---:|---:|---:|---:|
| TSAI | 1 | 0.223042846 | 0.038995623 | 1 | 1 |
| PARK | 1 | 0.224770003 | 0.038695762 | 1 | 0 |
| HORAUD | 1 | 0.224785260 | 0.038695514 | 1 | 0 |
| ANDREFF | 1 | 0.260862776 | 0.038716531 | 1 | 0 |
| DANIILIDIS | 1 | 0.224333122 | 0.039044823 | 2 | 0 |

选择规则：在刚体矩阵有效的结果中，按固定靶标平均平移残差最小选择，平均旋转残差用于同值决胜。

## 逐帧固定靶标残差

| Frame | 平移残差 (mm) | 旋转残差 (deg) | 异常 |
|---:|---:|---:|---:|
| 1 | 0.263172964 | 0.020487819 | 否 |
| 2 | 0.100733834 | 0.023740102 | 否 |
| 3 | 0.211634929 | 0.038624069 | 否 |
| 4 | 0.157250489 | 0.037167494 | 否 |
| 5 | 0.145873822 | 0.027063111 | 否 |
| 6 | 0.133119456 | 0.021577442 | 否 |
| 7 | 0.141625300 | 0.020141891 | 否 |
| 8 | 0.188189965 | 0.023041826 | 否 |
| 9 | 0.123377341 | 0.010345869 | 否 |
| 10 | 0.179691271 | 0.018616891 | 否 |
| 11 | 0.199908301 | 0.030611750 | 否 |
| 12 | 0.114626317 | 0.020679903 | 否 |
| 13 | 0.045529744 | 0.015960818 | 否 |
| 14 | 0.100712638 | 0.012636213 | 否 |
| 15 | 0.249130115 | 0.031177853 | 否 |
| 16 | 0.255098399 | 0.039545369 | 否 |
| 17 | 0.231468581 | 0.029510205 | 否 |
| 18 | 0.131950242 | 0.007543425 | 否 |
| 19 | 0.110482377 | 0.032658353 | 否 |
| 20 | 0.173226736 | 0.019114885 | 否 |
| 21 | 0.144826395 | 0.054046766 | 否 |
| 22 | 0.228951928 | 0.038288951 | 否 |
| 23 | 0.097191848 | 0.019426906 | 否 |
| 24 | 0.153102764 | 0.020485791 | 否 |
| 25 | 0.139020302 | 0.017474557 | 否 |
| 26 | 0.150150898 | 0.009621084 | 否 |
| 27 | 0.068675797 | 0.015511288 | 否 |
| 28 | 0.169128768 | 0.024827810 | 否 |
| 29 | 0.139676850 | 0.028631801 | 否 |
| 30 | 0.181814141 | 0.050108921 | 否 |
| 31 | 0.149853486 | 0.015434348 | 否 |
| 32 | 0.149710727 | 0.022297209 | 否 |
| 33 | 0.207080893 | 0.050848685 | 否 |
| 34 | 0.295377706 | 0.074265774 | 否 |
| 35 | 0.290309596 | 0.073002791 | 否 |
| 36 | 0.184837121 | 0.060563708 | 否 |
| 37 | 0.200321142 | 0.079354854 | 否 |
| 38 | 0.286152598 | 0.065584152 | 否 |
| 39 | 0.243158445 | 0.074015781 | 否 |
| 40 | 0.235424127 | 0.068215974 | 否 |
| 41 | 0.344116841 | 0.077392209 | 否 |
| 43 | 0.368013880 | 0.054068038 | 否 |
| 44 | 0.352599274 | 0.068011773 | 否 |
| 45 | 0.350712469 | 0.036582991 | 否 |
| 46 | 0.486730585 | 0.042988619 | 否 |
| 47 | 0.473501516 | 0.048631695 | 否 |
| 48 | 0.403382537 | 0.067147484 | 否 |
| 49 | 0.360081740 | 0.051079443 | 否 |
| 50 | 0.322653365 | 0.045002717 | 否 |
| 51 | 0.209496936 | 0.020778959 | 否 |
| 52 | 0.242024191 | 0.036993604 | 否 |
| 53 | 0.303670326 | 0.016425049 | 否 |
| 54 | 0.368831105 | 0.041070132 | 否 |
| 55 | 0.327970666 | 0.081200151 | 否 |
| 56 | 0.417253713 | 0.028184588 | 否 |
| 57 | 0.211446223 | 0.025483856 | 否 |
| 58 | 0.423303067 | 0.051722117 | 否 |
| 59 | 0.495671243 | 0.106465434 | 是 |
| 60 | 0.385075090 | 0.067688371 | 否 |
| 61 | 0.377597049 | 0.062923442 | 否 |
| 62 | 0.264465002 | 0.036234795 | 否 |
| 63 | 0.279694502 | 0.043609977 | 否 |
| 64 | 0.232627454 | 0.036218845 | 否 |
| 65 | 0.107565394 | 0.027731642 | 否 |
| 66 | 0.100316650 | 0.026664083 | 否 |
| 67 | 0.112480358 | 0.046575592 | 否 |
| 68 | 0.116344297 | 0.020212402 | 否 |
| 69 | 0.178806822 | 0.038077154 | 否 |
| 70 | 0.123125834 | 0.041174817 | 否 |
| 71 | 0.153073151 | 0.051610256 | 否 |
| 72 | 0.133325135 | 0.037315068 | 否 |
| 73 | 0.204963637 | 0.033215208 | 否 |
| 74 | 0.164282950 | 0.035485912 | 否 |
| 75 | 0.138399271 | 0.039457201 | 否 |

## 标定输出文件（绝对路径）

- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000001.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000002.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000003.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000004.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000005.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000006.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000007.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000008.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000009.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000010.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000011.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000012.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000013.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000014.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000015.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000016.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000017.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000018.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000019.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000020.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000021.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000022.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000023.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000024.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000025.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000026.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000027.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000028.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000029.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000030.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000031.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000032.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000033.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000034.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000035.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000036.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000037.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000038.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000039.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000040.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000041.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000043.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000044.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000045.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000046.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000047.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000048.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000049.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000050.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000051.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000052.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000053.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000054.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000055.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000056.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000057.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000058.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000059.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000060.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000061.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000062.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000063.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000064.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000065.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000066.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000067.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000068.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000069.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000070.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000071.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000072.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000073.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000074.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/DetectedCorners/DetectedCorners_000075.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/FixedTargetResiduals_ANDREFF.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/FixedTargetResiduals_DANIILIDIS.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/FixedTargetResiduals_HORAUD.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/FixedTargetResiduals_PARK.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/FixedTargetResiduals_TSAI.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/FixedTargetRotationResidual_ANDREFF.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/FixedTargetRotationResidual_DANIILIDIS.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/FixedTargetRotationResidual_HORAUD.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/FixedTargetRotationResidual_PARK.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/FixedTargetRotationResidual_TSAI.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/FixedTargetTranslationResidual_ANDREFF.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/FixedTargetTranslationResidual_DANIILIDIS.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/FixedTargetTranslationResidual_HORAUD.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/FixedTargetTranslationResidual_PARK.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/FixedTargetTranslationResidual_TSAI.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/T_cam2gripper_ANDREFF.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/T_cam2gripper_DANIILIDIS.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/T_cam2gripper_HORAUD.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/T_cam2gripper_PARK.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/T_cam2gripper_TSAI.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/T_cam2gripper_selected.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/T_gripper2cam_ANDREFF.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/T_gripper2cam_DANIILIDIS.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/T_gripper2cam_HORAUD.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/T_gripper2cam_PARK.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/T_gripper2cam_TSAI.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini/FinalTransforms/handeye_method_summary.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini/GeminiInputIntrinsics.npz`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/corner_detection_results.csv`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000001.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000002.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000003.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000004.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000005.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000006.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000007.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000008.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000009.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000010.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000011.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000012.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000013.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000014.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000015.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000016.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000017.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000018.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000019.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000020.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000021.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000022.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000023.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000024.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000025.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000026.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000027.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000028.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000029.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000030.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000031.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000032.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000033.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000034.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000035.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000036.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000037.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000038.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000039.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000040.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000041.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000042.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000043.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000044.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000045.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000046.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000047.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000048.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000049.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000050.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000051.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000052.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000053.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000054.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000055.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000056.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000057.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000058.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000059.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000060.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000061.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000062.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000063.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000064.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000065.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000066.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000067.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000068.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000069.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000070.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000071.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000072.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000073.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000074.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/diagnostics/corners/frame-000075.color_corners.png`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000001.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000002.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000003.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000004.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000005.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000006.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000007.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000008.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000009.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000010.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000011.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000012.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000013.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000014.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000015.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000016.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000017.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000018.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000019.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000020.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000021.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000022.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000023.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000024.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000025.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000026.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000027.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000028.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000029.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000030.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000031.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000032.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000033.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000034.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000035.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000036.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000037.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000038.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000039.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000040.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000041.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000042.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000043.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000044.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000045.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000046.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000047.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000048.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000049.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000050.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000051.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000052.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000053.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000054.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000055.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000056.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000057.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000058.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000059.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000060.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000061.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000062.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000063.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000064.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000065.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000066.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000067.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000068.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000069.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000070.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000071.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000072.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000073.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000074.npy`
- `/home/root01/ramanseg/calibration/handeye_gemini/robot_poses/frame-000075.npy`

## 日志与机器可读统计（绝对路径）

- `/home/root01/ramanseg/reports/handeye_gemini/calibration_report.md`
- `/home/root01/ramanseg/reports/handeye_gemini/calibration_run.log`
- `/home/root01/ramanseg/reports/handeye_gemini/fusion_report.md`
- `/home/root01/ramanseg/reports/handeye_gemini/fusion_run.log`
- `/home/root01/ramanseg/reports/handeye_gemini/fusion_stats.json`
- `/home/root01/ramanseg/reports/handeye_gemini/pose_generation.log`
- `/home/root01/ramanseg/reports/handeye_gemini/pose_validation.json`

- 本报告：`/home/root01/ramanseg/reports/handeye_gemini/calibration_report.md`
- 适配标定脚本：`/home/root01/ramanseg/scripts/HandEyeCalibration_class_fixed.py`
- 原始标定脚本（未覆盖）：`/home/root01/HandEyeCalibration/HandEyeCalibration-using-OpenCV/HandEyeCalibration_class_fixed.py`
