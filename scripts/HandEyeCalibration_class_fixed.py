import cv2
import numpy as np
import glob
import matplotlib.pyplot as plt
import os
import re
import json
from pathlib import Path


class CameraCalibration:
    """Camera intrinsic calibration and eye-in-hand calibration.

    Assumptions used by this version:
    1. The camera is fixed on the robot end effector.
    2. The checkerboard target is fixed in the external environment.
    3. Each .npy pose is a 4x4 transform describing the gripper/end-effector
       pose in the robot base frame, i.e. OpenCV's gripper2base transform.
    4. Translation in the .npy files and square_size both use metres.
    """

    def __init__(
        self,
        image_folder,
        transforms_folder,
        pattern_size=(11, 8),
        square_size=15 / 1000,
        ShowProjectError=False,
        ShowCorners=False,
        RunRobotWorldHandEye=True,
        intrinsic_matrix=None,
        distortion_coefficients=None,
        output_root=".",
        expected_valid_frame_ids=None,
    ):
        self.pattern_size = pattern_size
        self.square_size = square_size
        self.output_root = os.path.abspath(output_root)
        self.final_transforms_dir = os.path.join(
            self.output_root, "FinalTransforms"
        )
        self.detected_corners_dir = os.path.join(
            self.output_root, "DetectedCorners"
        )

        # MODIFIED: Match images and robot poses by frame number instead of
        # relying only on two independently sorted file lists.
        self.image_files, self.transform_files, self.frame_ids = (
            self.match_image_and_transform_files(
                image_folder,
                transforms_folder,
            )
        )

        # MODIFIED: cv2.imread already returns BGR, so the unnecessary
        # COLOR_RGB2BGR conversion in the original code was removed.
        self.images = [cv2.imread(path) for path in self.image_files]
        for path, image in zip(self.image_files, self.images):
            if image is None:
                raise ValueError("Failed to read image: {}".format(path))

        # MODIFIED: .npy returns an ndarray directly. Do not use ['arr_0'],
        # which is only appropriate for unnamed arrays stored in .npz files.
        self.All_T_gripper2base_list = [
            self.load_and_validate_transform(path)
            for path in self.transform_files
        ]

        print("Matched image-pose pairs:", len(self.image_files))
        print("Chessboard pattern_size:", self.pattern_size)
        print("Chessboard square_size/m:", self.square_size)

        self.chessboard_corners, self.IndexWithImg = (
            self.find_chessboard_corners(
                self.images,
                self.pattern_size,
                ShowCorners=ShowCorners,
            )
        )

        if len(self.chessboard_corners) < 3:
            raise ValueError(
                "Only {} valid checkerboard images were found; at least "
                "3 are required and 10-15 or more are recommended.".format(
                    len(self.chessboard_corners)
                )
            )

        if intrinsic_matrix is None or distortion_coefficients is None:
            # Retained for compatibility with the original program. The
            # RamanSeg entry point always supplies Gemini factory intrinsics.
            image_size = (
                self.images[0].shape[1],
                self.images[0].shape[0],
            )
            (
                self.intrinsic_matrix,
                self.distortion_coefficients,
                self.calibration_rms,
            ) = self.calculate_intrinsics(
                self.chessboard_corners,
                self.pattern_size,
                self.square_size,
                image_size,
                ShowProjectError=ShowProjectError,
            )
        else:
            self.intrinsic_matrix = np.asarray(
                intrinsic_matrix, dtype=np.float64
            )
            self.distortion_coefficients = np.asarray(
                distortion_coefficients, dtype=np.float64
            ).reshape(-1)
            if self.intrinsic_matrix.shape != (3, 3):
                raise ValueError("External intrinsic matrix must be 3x3")
            if self.distortion_coefficients.size != 5:
                raise ValueError(
                    "External OpenCV distortion vector must have 5 values"
                )
            self.calibration_rms = np.nan
            print("Using external Gemini color-camera intrinsics; "
                  "cv2.calibrateCamera is skipped.")
            print("Intrinsic matrix:\n", self.intrinsic_matrix)
            print("Distortion coefficients:\n", self.distortion_coefficients)

        # MODIFIED: Remove the robot poses corresponding to images in which
        # checkerboard detection failed. Image-pose indexing stays aligned.
        self.T_gripper2base_list = [
            self.All_T_gripper2base_list[index]
            for index in self.IndexWithImg
        ]

        self.valid_frame_ids = [
            self.frame_ids[index]
            for index in self.IndexWithImg
        ]
        if expected_valid_frame_ids is not None:
            expected = list(expected_valid_frame_ids)
            if self.valid_frame_ids != expected:
                raise ValueError(
                    "Checkerboard-valid frame IDs do not match expectation. "
                    "Expected {}, got {}".format(
                        expected, self.valid_frame_ids
                    )
                )

        os.makedirs(self.output_root, exist_ok=True)
        np.savez(
            os.path.join(self.output_root, "GeminiInputIntrinsics.npz"),
            intrinsic_matrix=self.intrinsic_matrix,
            distortion_coefficients=self.distortion_coefficients,
            calibration_rms=np.array(self.calibration_rms),
        )

        # solvePnP returns target2cam directly, which is exactly the direction
        # required by cv2.calibrateHandEye.
        self.RTarget2Cam, self.TTarget2Cam = self.compute_camera_poses(
            self.chessboard_corners,
            self.pattern_size,
            self.square_size,
            self.intrinsic_matrix,
            self.distortion_coefficients,
        )

        self.T_target2cam_list = [
            self.make_transform(R, t)
            for R, t in zip(self.RTarget2Cam, self.TTarget2Cam)
        ]

        # MODIFIED: The Excel-derived pose describes the end effector in the
        # robot base frame, so it is OpenCV's gripper2base input and must be
        # passed directly to calibrateHandEye without inversion.
        self.R_gripper2base = [
            T[:3, :3]
            for T in self.T_gripper2base_list
        ]
        self.t_gripper2base = [
            T[:3, 3].reshape(3, 1)
            for T in self.T_gripper2base_list
        ]

        # MODIFIED: Fix the original unexpected indentation by keeping the
        # output-directory creation at the same indentation level as the rest
        # of __init__.
        os.makedirs(self.final_transforms_dir, exist_ok=True)

        self.handeye_results = {}
        self.run_hand_eye_calibration()
        self.select_best_handeye_result()

        if RunRobotWorldHandEye:
            self.run_robot_world_hand_eye_calibration()

    @staticmethod
    def extract_frame_id(path):
        """Extract frame number from names such as frame-000001.color.png."""
        match = re.search(
            r"frame-(\d+)",
            os.path.basename(path),
            flags=re.IGNORECASE,
        )
        if match is None:
            raise ValueError(
                "Cannot extract frame id from file name: {}".format(path)
            )
        return int(match.group(1))

    def match_image_and_transform_files(
        self,
        image_folder,
        transforms_folder,
    ):
        image_candidates = glob.glob(
            os.path.join(image_folder, "*.png")
        )
        transform_candidates = glob.glob(
            os.path.join(transforms_folder, "*.npy")
        )

        if not image_candidates:
            raise ValueError(
                "No PNG images found in: {}".format(image_folder)
            )
        if not transform_candidates:
            raise ValueError(
                "No NPY robot poses found in: {}".format(
                    transforms_folder
                )
            )

        image_map = {}
        for path in image_candidates:
            frame_id = self.extract_frame_id(path)
            if frame_id in image_map:
                raise ValueError(
                    "Duplicate image frame id {}: {} and {}".format(
                        frame_id,
                        image_map[frame_id],
                        path,
                    )
                )
            image_map[frame_id] = path

        transform_map = {}
        for path in transform_candidates:
            frame_id = self.extract_frame_id(path)
            if frame_id in transform_map:
                raise ValueError(
                    "Duplicate transform frame id {}: {} and {}".format(
                        frame_id,
                        transform_map[frame_id],
                        path,
                    )
                )
            transform_map[frame_id] = path

        image_ids = set(image_map.keys())
        transform_ids = set(transform_map.keys())

        missing_transforms = sorted(image_ids - transform_ids)
        missing_images = sorted(transform_ids - image_ids)

        if missing_transforms or missing_images:
            raise ValueError(
                "Image-pose frame mismatch. Missing transforms for frames: "
                "{}. Missing images for frames: {}".format(
                    missing_transforms,
                    missing_images,
                )
            )

        frame_ids = sorted(image_ids)
        image_files = [image_map[i] for i in frame_ids]
        transform_files = [transform_map[i] for i in frame_ids]
        return image_files, transform_files, frame_ids

    @staticmethod
    def load_and_validate_transform(path):
        T = np.asarray(np.load(path), dtype=np.float64)

        if T.shape != (4, 4):
            raise ValueError(
                "Robot pose must be a 4x4 matrix: {}, shape={}".format(
                    path,
                    T.shape,
                )
            )

        if not np.all(np.isfinite(T)):
            raise ValueError(
                "Robot pose contains NaN or Inf: {}".format(path)
            )

        if not np.allclose(
            T[3, :],
            np.array([0.0, 0.0, 0.0, 1.0]),
            atol=1e-6,
        ):
            raise ValueError(
                "Invalid homogeneous last row in {}: {}".format(
                    path,
                    T[3, :],
                )
            )

        R = T[:3, :3]
        if not np.allclose(
            R.T.dot(R),
            np.eye(3),
            atol=1e-3,
        ):
            raise ValueError(
                "Rotation is not orthonormal in: {}".format(path)
            )

        determinant = np.linalg.det(R)
        if not np.isclose(determinant, 1.0, atol=1e-3):
            raise ValueError(
                "Rotation determinant is not 1 in {}: {}".format(
                    path,
                    determinant,
                )
            )

        return T

    @staticmethod
    def make_transform(R, t):
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = np.asarray(R, dtype=np.float64)
        T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
        return T

    def find_chessboard_corners(
        self,
        images,
        pattern_size,
        ShowCorners=False,
    ):
        chessboard_corners = []
        indices_with_image = []

        os.makedirs(self.detected_corners_dir, exist_ok=True)

        print("Finding corners...")
        for index, image in enumerate(images):
            image_name = os.path.basename(self.image_files[index])
            print(
                "[{}/{}] Detecting {}...".format(
                    index + 1,
                    len(images),
                    image_name,
                ),
                flush=True,
            )

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            ret = False
            corners = None
            detector = "NONE"

            if hasattr(cv2, "findChessboardCornersSB"):
                ret, corners = cv2.findChessboardCornersSB(
                    gray,
                    pattern_size,
                    flags=(
                        cv2.CALIB_CB_NORMALIZE_IMAGE
                        | cv2.CALIB_CB_EXHAUSTIVE
                        | cv2.CALIB_CB_ACCURACY
                    ),
                )
                if ret:
                    detector = "SB"

            if not ret:
                ret, corners = cv2.findChessboardCorners(
                    gray,
                    pattern_size,
                    flags=(
                        cv2.CALIB_CB_ADAPTIVE_THRESH
                        | cv2.CALIB_CB_NORMALIZE_IMAGE
                    ),
                )
                if ret:
                    detector = "CLASSIC"

            if not ret:
                print(
                    "No chessboard found in image: {}".format(index),
                    flush=True,
                )
                continue

            # MODIFIED: Refine detected corners to sub-pixel precision before
            # intrinsic calibration and solvePnP.
            criteria = (
                cv2.TERM_CRITERIA_EPS
                | cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.001,
            )
            corners = cv2.cornerSubPix(
                gray,
                np.asarray(corners, dtype=np.float32),
                (11, 11),
                (-1, -1),
                criteria,
            )

            # For this board, OpenCV's classic fallback enumerates the two
            # fallback detections (frames 33 and 41) in the opposite direction
            # to findChessboardCornersSB. Reverse the full grid so the same
            # object point is paired with the same physical corner.
            if detector == "CLASSIC":
                corners = corners[::-1].copy()

            expected_corner_count = pattern_size[0] * pattern_size[1]
            if len(corners) != expected_corner_count:
                raise ValueError(
                    "Frame {} returned {} corners instead of {}".format(
                        self.frame_ids[index],
                        len(corners),
                        expected_corner_count,
                    )
                )

            print(
                "Chessboard found in image: {}, corners={}, detector={}".format(
                    index,
                    len(corners),
                    detector,
                ),
                flush=True,
            )

            chessboard_corners.append(corners)
            indices_with_image.append(index)

            drawn = image.copy()
            cv2.drawChessboardCorners(
                drawn,
                pattern_size,
                corners,
                ret,
            )

            output_path = os.path.join(
                self.detected_corners_dir,
                "DetectedCorners_{:06d}.png".format(
                    self.frame_ids[index]
                ),
            )
            cv2.imwrite(output_path, drawn)

            if ShowCorners:
                plt.figure()
                plt.imshow(cv2.cvtColor(drawn, cv2.COLOR_BGR2RGB))
                plt.title("Detected corners: {}".format(image_name))
                plt.axis("off")
                plt.show()

        print(
            "Corner detection finished: {}/{} images succeeded".format(
                len(chessboard_corners),
                len(images),
            ),
            flush=True,
        )
        return chessboard_corners, indices_with_image

    @staticmethod
    def create_object_points(pattern_size, square_size):
        object_points = np.zeros(
            (pattern_size[0] * pattern_size[1], 3),
            dtype=np.float32,
        )
        object_points[:, :2] = (
            np.mgrid[
                0:pattern_size[0],
                0:pattern_size[1],
            ].T.reshape(-1, 2)
            * square_size
        )
        return object_points

    def calculate_intrinsics(
        self,
        chessboard_corners,
        pattern_size,
        square_size,
        image_size,
        ShowProjectError=False,
    ):
        object_template = self.create_object_points(
            pattern_size,
            square_size,
        )
        object_points = [
            object_template.copy()
            for _ in chessboard_corners
        ]

        rms, matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
            object_points,
            chessboard_corners,
            image_size,
            None,
            None,
        )

        reprojection_rmse = self.calculate_reprojection_error(
            object_points,
            chessboard_corners,
            rvecs,
            tvecs,
            matrix,
            distortion,
            ShowPlot=ShowProjectError,
        )

        print("OpenCV calibration RMS/pixel:", rms)
        print("Reprojection RMSE/pixel:", reprojection_rmse)
        print("Intrinsic matrix:\n", matrix)
        print("Distortion coefficients:\n", distortion)
        return matrix, distortion, rms

    def compute_camera_poses(
        self,
        chessboard_corners,
        pattern_size,
        square_size,
        intrinsic_matrix,
        distortion_coefficients,
    ):
        object_points = self.create_object_points(
            pattern_size,
            square_size,
        )

        rotations_target2cam = []
        translations_target2cam = []

        for index, corners in enumerate(chessboard_corners):
            success, rvec, tvec = cv2.solvePnP(
                object_points,
                corners,
                intrinsic_matrix,
                distortion_coefficients,
            )
            if not success:
                raise RuntimeError(
                    "solvePnP failed for valid sample index {}".format(
                        index
                    )
                )

            R, _ = cv2.Rodrigues(rvec)
            rotations_target2cam.append(R)
            translations_target2cam.append(
                np.asarray(tvec, dtype=np.float64).reshape(3, 1)
            )

        return rotations_target2cam, translations_target2cam

    @staticmethod
    def calculate_reprojection_error(
        object_points,
        image_points,
        rvecs,
        tvecs,
        matrix,
        distortion,
        ShowPlot=False,
    ):
        squared_error_sum = 0.0
        point_count = 0
        per_image_rmse = []

        for index in range(len(object_points)):
            projected, _ = cv2.projectPoints(
                object_points[index],
                rvecs[index],
                tvecs[index],
                matrix,
                distortion,
            )

            residual = (
                image_points[index].reshape(-1, 2)
                - projected.reshape(-1, 2)
            )
            squared_distances = np.sum(residual ** 2, axis=1)

            per_image_rmse.append(
                float(np.sqrt(np.mean(squared_distances)))
            )
            squared_error_sum += float(np.sum(squared_distances))
            point_count += residual.shape[0]

        total_rmse = np.sqrt(squared_error_sum / point_count)

        if ShowPlot:
            # MODIFIED: Save and close the plot instead of blocking execution
            # with plt.show() on a headless Linux server.
            fig, ax = plt.subplots()
            image_indices = range(1, len(per_image_rmse) + 1)
            ax.bar(image_indices, per_image_rmse)
            ax.set_xlabel("Valid image index")
            ax.set_ylabel("Reprojection RMSE (pixel)")
            ax.set_title("Reprojection Error for Each Image")
            fig.tight_layout()
            fig.savefig(
                os.path.join(self.output_root, "ReprojectionError.png"),
                dpi=200,
                bbox_inches="tight",
            )
            plt.close(fig)
            print("Per-image reprojection RMSE:", per_image_rmse)

        return float(total_rmse)

    def run_hand_eye_calibration(self):
        """Run several OpenCV hand-eye methods and diagnose each result."""
        methods = [
            ("TSAI", cv2.CALIB_HAND_EYE_TSAI),
            ("PARK", cv2.CALIB_HAND_EYE_PARK),
            ("HORAUD", cv2.CALIB_HAND_EYE_HORAUD),
            ("ANDREFF", cv2.CALIB_HAND_EYE_ANDREFF),
            ("DANIILIDIS", cv2.CALIB_HAND_EYE_DANIILIDIS),
        ]

        print("\nRunning cv2.calibrateHandEye...")

        for method_name, method_flag in methods:
            # OpenCV expects gripper2base first and target2cam second.
            try:
                R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                    self.R_gripper2base,
                    self.t_gripper2base,
                    self.RTarget2Cam,
                    self.TTarget2Cam,
                    method=method_flag,
                )
            except cv2.error as error:
                print(
                    "Method {} failed in cv2.calibrateHandEye: {}".format(
                        method_name, error
                    )
                )
                self.handeye_results[method_name] = {
                    "valid": False,
                    "error": str(error),
                }
                continue

            T_cam2gripper = self.make_transform(
                R_cam2gripper,
                t_cam2gripper,
            )
            R_result = T_cam2gripper[:3, :3]
            result_is_valid = (
                np.all(np.isfinite(T_cam2gripper))
                and np.allclose(R_result.T.dot(R_result), np.eye(3), atol=1e-3)
                and np.isclose(np.linalg.det(R_result), 1.0, atol=1e-3)
            )
            if not result_is_valid:
                print("Method {} produced an invalid rigid transform; "
                      "it is excluded from selection.".format(method_name))
                self.handeye_results[method_name] = {
                    "valid": False,
                    "T": T_cam2gripper,
                }
                continue
            T_gripper2cam = np.linalg.inv(T_cam2gripper)

            print("\n" + "=" * 80)
            print("Method:", method_name)
            print("=" * 80)
            print("R_cam2gripper:\n", R_cam2gripper)
            print("t_cam2gripper/m:\n", t_cam2gripper)

            consistency = self.evaluate_fixed_target_consistency(
                T_cam2gripper
            )

            mean_translation_error_mm = (
                consistency["mean_translation_residual_mm"]
            )
            mean_rotation_error_deg = (
                consistency["mean_rotation_residual_deg"]
            )

            print(
                "\nFixed-target mean translation residual/mm:",
                mean_translation_error_mm,
            )
            print(
                "Fixed-target median translation residual/mm:",
                consistency["median_translation_residual_mm"],
            )
            print(
                "Fixed-target max translation residual/mm:",
                consistency["max_translation_residual_mm"],
            )

            print(
                "\nFixed-target mean rotation residual/deg:",
                mean_rotation_error_deg,
            )
            print(
                "Fixed-target median rotation residual/deg:",
                consistency["median_rotation_residual_deg"],
            )
            print(
                "Fixed-target max rotation residual/deg:",
                consistency["max_rotation_residual_deg"],
            )

            print(
                "\nRobust translation outlier threshold/mm:",
                consistency["translation_threshold_mm"],
            )
            print(
                "Robust rotation outlier threshold/deg:",
                consistency["rotation_threshold_deg"],
            )

            print("\nPer-frame fixed-target residuals:")
            print(
                "{:<12s} {:>18s} {:>18s} {:>12s} {:>12s} {:>12s}".format(
                    "Frame",
                    "Translation/mm",
                    "Rotation/deg",
                    "T-outlier",
                    "R-outlier",
                    "Status",
                )
            )
            print("-" * 92)

            for (
                frame_id,
                trans_error,
                rot_error,
                trans_outlier,
                rot_outlier,
                is_outlier,
            ) in zip(
                self.valid_frame_ids,
                consistency["translation_residuals_mm"],
                consistency["rotation_residuals_deg"],
                consistency["translation_outlier_mask"],
                consistency["rotation_outlier_mask"],
                consistency["outlier_mask"],
            ):
                status = "OUTLIER" if is_outlier else "OK"
                print(
                    "{:<12d} {:>18.6f} {:>18.6f} {:>12s} {:>12s} {:>12s}".format(
                        int(frame_id),
                        float(trans_error),
                        float(rot_error),
                        "YES" if trans_outlier else "NO",
                        "YES" if rot_outlier else "NO",
                        status,
                    )
                )

            outlier_frame_ids = consistency["outlier_frame_ids"]
            if len(outlier_frame_ids) > 0:
                print(
                    "\nSuspected abnormal frame IDs:",
                    outlier_frame_ids.tolist(),
                )
            else:
                print(
                    "\nNo obvious abnormal frame was detected "
                    "by the MAD criterion."
                )

            self.save_fixed_target_residuals(
                method_name,
                consistency,
            )
            self.plot_fixed_target_residuals(
                method_name,
                consistency,
            )

            np.savez(
                os.path.join(
                    self.final_transforms_dir,
                    "T_cam2gripper_{}.npz".format(method_name),
                ),
                T=T_cam2gripper,
                method=np.array(method_name),
                fixed_target_translation_residual_mm=np.array(
                    mean_translation_error_mm
                ),
                fixed_target_rotation_residual_deg=np.array(
                    mean_rotation_error_deg
                ),
                translation_residuals_mm=np.asarray(
                    consistency["translation_residuals_mm"]
                ),
                rotation_residuals_deg=np.asarray(
                    consistency["rotation_residuals_deg"]
                ),
                translation_threshold_mm=np.array(
                    consistency["translation_threshold_mm"]
                ),
                rotation_threshold_deg=np.array(
                    consistency["rotation_threshold_deg"]
                ),
                translation_outlier_mask=np.asarray(
                    consistency["translation_outlier_mask"],
                    dtype=bool,
                ),
                rotation_outlier_mask=np.asarray(
                    consistency["rotation_outlier_mask"],
                    dtype=bool,
                ),
                outlier_mask=np.asarray(
                    consistency["outlier_mask"],
                    dtype=bool,
                ),
                outlier_frame_ids=np.asarray(
                    consistency["outlier_frame_ids"],
                    dtype=int,
                ),
                valid_frame_ids=np.asarray(
                    self.valid_frame_ids,
                    dtype=int,
                ),
            )

            np.savez(
                os.path.join(
                    self.final_transforms_dir,
                    "T_gripper2cam_{}.npz".format(method_name),
                ),
                T=T_gripper2cam,
                method=np.array(method_name),
                valid_frame_ids=np.asarray(
                    self.valid_frame_ids,
                    dtype=int,
                ),
            )

            self.handeye_results[method_name] = {
                "valid": True,
                "T": T_cam2gripper,
                "consistency": consistency,
            }

    def select_best_handeye_result(self):
        """Select the valid result with the lowest fixed-target residuals."""
        valid_results = [
            (name, result)
            for name, result in self.handeye_results.items()
            if result.get("valid", False)
        ]
        if not valid_results:
            raise RuntimeError("No hand-eye method produced a valid transform")

        # Fixed-target mean translation is the primary consistency criterion;
        # mean rotation is the deterministic tie-breaker.
        method_name, result = min(
            valid_results,
            key=lambda item: (
                item[1]["consistency"]["mean_translation_residual_mm"],
                item[1]["consistency"]["mean_rotation_residual_deg"],
            ),
        )
        consistency = result["consistency"]
        selected_path = os.path.join(
            self.final_transforms_dir,
            "T_cam2gripper_selected.npz",
        )
        np.savez(
            selected_path,
            T=result["T"],
            method=np.array(method_name),
            selection_rule=np.array(
                "lowest mean fixed-target translation residual; "
                "rotation residual tie-breaker"
            ),
            fixed_target_translation_residual_mm=np.array(
                consistency["mean_translation_residual_mm"]
            ),
            fixed_target_rotation_residual_deg=np.array(
                consistency["mean_rotation_residual_deg"]
            ),
            translation_residuals_mm=np.asarray(
                consistency["translation_residuals_mm"]
            ),
            rotation_residuals_deg=np.asarray(
                consistency["rotation_residuals_deg"]
            ),
            translation_threshold_mm=np.array(
                consistency["translation_threshold_mm"]
            ),
            rotation_threshold_deg=np.array(
                consistency["rotation_threshold_deg"]
            ),
            outlier_mask=np.asarray(consistency["outlier_mask"], dtype=bool),
            outlier_frame_ids=np.asarray(
                consistency["outlier_frame_ids"], dtype=int
            ),
            valid_frame_ids=np.asarray(self.valid_frame_ids, dtype=int),
        )

        summary_path = os.path.join(
            self.final_transforms_dir, "handeye_method_summary.csv"
        )
        with open(summary_path, "w", encoding="utf-8") as stream:
            stream.write(
                "method,valid,mean_translation_residual_mm,"
                "mean_rotation_residual_deg,outlier_count,selected\n"
            )
            for name, candidate in self.handeye_results.items():
                if candidate.get("valid", False):
                    c = candidate["consistency"]
                    stream.write(
                        "{},{},{:.9f},{:.9f},{},{}\n".format(
                            name,
                            1,
                            c["mean_translation_residual_mm"],
                            c["mean_rotation_residual_deg"],
                            len(c["outlier_frame_ids"]),
                            int(name == method_name),
                        )
                    )
                else:
                    stream.write("{},0,nan,nan,0,0\n".format(name))

        self.selected_method = method_name
        self.selected_T_cam2gripper = result["T"]
        self.selected_consistency = consistency
        self.selected_transform_path = selected_path
        print("\nSelected hand-eye method:", method_name)
        print("Selected T_cam2gripper:\n", result["T"])
        print("Saved selected hand-eye transform:", selected_path)

    def evaluate_fixed_target_consistency(self, T_cam2gripper):
        """Evaluate how constant the fixed checkerboard is in base frame.

        For every valid frame:

            T_target2base =
                T_gripper2base @ T_cam2gripper @ T_target2cam

        A correct eye-in-hand calibration with synchronized robot/image data
        should make T_target2base nearly constant over all frames.
        """
        target_poses_in_base = []

        # 1. Transform the checkerboard pose into the robot base frame.
        for T_gripper2base, T_target2cam in zip(
            self.T_gripper2base_list,
            self.T_target2cam_list,
        ):
            T_target2base = (
                T_gripper2base
                .dot(T_cam2gripper)
                .dot(T_target2cam)
            )
            target_poses_in_base.append(T_target2base)

        # 2. Translation consistency.
        translations = np.asarray(
            [T[:3, 3] for T in target_poses_in_base],
            dtype=np.float64,
        )
        mean_translation = np.mean(translations, axis=0)
        translation_residuals_m = np.linalg.norm(
            translations - mean_translation,
            axis=1,
        )
        translation_residuals_mm = translation_residuals_m * 1000.0

        # 3. Rotation consistency. Use an average rotation as the reference
        # instead of arbitrarily using the first frame.
        rotations = [
            T[:3, :3]
            for T in target_poses_in_base
        ]
        reference_rotation = self.average_rotation_matrix(rotations)

        rotation_residuals_deg = []
        for R in rotations:
            delta_R = reference_rotation.T.dot(R)
            cosine = np.clip(
                (np.trace(delta_R) - 1.0) / 2.0,
                -1.0,
                1.0,
            )
            angle_deg = np.degrees(np.arccos(cosine))
            rotation_residuals_deg.append(float(angle_deg))

        rotation_residuals_deg = np.asarray(
            rotation_residuals_deg,
            dtype=np.float64,
        )

        # 4. Robust outlier thresholds using median + 3 * scaled MAD.
        translation_threshold_mm = self.robust_upper_threshold(
            translation_residuals_mm
        )
        rotation_threshold_deg = self.robust_upper_threshold(
            rotation_residuals_deg
        )

        translation_outliers = (
            translation_residuals_mm > translation_threshold_mm
        )
        rotation_outliers = (
            rotation_residuals_deg > rotation_threshold_deg
        )
        outlier_mask = np.logical_or(
            translation_outliers,
            rotation_outliers,
        )

        valid_frame_ids = np.asarray(
            self.valid_frame_ids,
            dtype=int,
        )
        outlier_frame_ids = valid_frame_ids[outlier_mask]

        result = {
            "target_poses_in_base": target_poses_in_base,
            "mean_translation_xyz_m": mean_translation,
            "translation_residuals_mm": translation_residuals_mm,
            "rotation_residuals_deg": rotation_residuals_deg,
            "mean_translation_residual_mm": float(
                np.mean(translation_residuals_mm)
            ),
            "median_translation_residual_mm": float(
                np.median(translation_residuals_mm)
            ),
            "max_translation_residual_mm": float(
                np.max(translation_residuals_mm)
            ),
            "mean_rotation_residual_deg": float(
                np.mean(rotation_residuals_deg)
            ),
            "median_rotation_residual_deg": float(
                np.median(rotation_residuals_deg)
            ),
            "max_rotation_residual_deg": float(
                np.max(rotation_residuals_deg)
            ),
            "translation_threshold_mm": float(
                translation_threshold_mm
            ),
            "rotation_threshold_deg": float(
                rotation_threshold_deg
            ),
            "translation_outlier_mask": translation_outliers,
            "rotation_outlier_mask": rotation_outliers,
            "outlier_mask": outlier_mask,
            "outlier_frame_ids": outlier_frame_ids,
        }
        return result

    @staticmethod
    def average_rotation_matrix(rotation_matrices):
        """Average rotation matrices and project the mean back onto SO(3)."""
        if len(rotation_matrices) == 0:
            raise ValueError("rotation_matrices must not be empty")

        M = np.zeros((3, 3), dtype=np.float64)
        for R in rotation_matrices:
            M += np.asarray(R, dtype=np.float64)
        M /= float(len(rotation_matrices))

        U, _, Vt = np.linalg.svd(M)
        R_mean = U.dot(Vt)

        # Enforce a proper rotation: det(R) = +1.
        if np.linalg.det(R_mean) < 0:
            U[:, -1] *= -1.0
            R_mean = U.dot(Vt)

        return R_mean

    @staticmethod
    def robust_upper_threshold(values, k=3.0):
        """Return median + k * 1.4826 * MAD as a robust upper threshold.

        If the scaled MAD is numerically zero, fall back to mean + k * std.
        """
        values = np.asarray(values, dtype=np.float64)
        if values.size == 0:
            raise ValueError("values must not be empty")

        median = np.median(values)
        mad = np.median(np.abs(values - median))
        scaled_mad = 1.4826 * mad

        if scaled_mad > 1e-12:
            threshold = median + k * scaled_mad
        else:
            threshold = np.mean(values) + k * np.std(values)

        return float(threshold)

    def save_fixed_target_residuals(
        self,
        method_name,
        consistency,
    ):
        """Save per-frame fixed-target residuals as a CSV file."""
        frame_ids = np.asarray(
            self.valid_frame_ids,
            dtype=int,
        )
        translation_residuals_mm = np.asarray(
            consistency["translation_residuals_mm"],
            dtype=np.float64,
        )
        rotation_residuals_deg = np.asarray(
            consistency["rotation_residuals_deg"],
            dtype=np.float64,
        )
        translation_outlier_mask = np.asarray(
            consistency["translation_outlier_mask"],
            dtype=bool,
        )
        rotation_outlier_mask = np.asarray(
            consistency["rotation_outlier_mask"],
            dtype=bool,
        )
        outlier_mask = np.asarray(
            consistency["outlier_mask"],
            dtype=bool,
        )

        output = np.column_stack(
            (
                frame_ids,
                translation_residuals_mm,
                rotation_residuals_deg,
                translation_outlier_mask.astype(int),
                rotation_outlier_mask.astype(int),
                outlier_mask.astype(int),
            )
        )

        output_path = os.path.join(
            self.final_transforms_dir,
            "FixedTargetResiduals_{}.csv".format(method_name),
        )

        np.savetxt(
            output_path,
            output,
            delimiter=",",
            header=(
                "frame_id,"
                "translation_residual_mm,"
                "rotation_residual_deg,"
                "translation_outlier,"
                "rotation_outlier,"
                "outlier"
            ),
            comments="",
            fmt=[
                "%d",
                "%.9f",
                "%.9f",
                "%d",
                "%d",
                "%d",
            ],
        )
        print("Saved per-frame residuals:", output_path)

    def plot_fixed_target_residuals(
        self,
        method_name,
        consistency,
    ):
        """Plot per-frame translation and rotation residuals."""
        frame_ids = np.asarray(
            self.valid_frame_ids,
            dtype=int,
        )
        translation_residuals_mm = np.asarray(
            consistency["translation_residuals_mm"],
            dtype=np.float64,
        )
        rotation_residuals_deg = np.asarray(
            consistency["rotation_residuals_deg"],
            dtype=np.float64,
        )
        translation_threshold_mm = consistency[
            "translation_threshold_mm"
        ]
        rotation_threshold_deg = consistency[
            "rotation_threshold_deg"
        ]
        outlier_mask = np.asarray(
            consistency["outlier_mask"],
            dtype=bool,
        )

        positions = np.arange(len(frame_ids))
        tick_step = max(
            1,
            int(np.ceil(len(frame_ids) / 20.0)),
        )
        tick_positions = positions[::tick_step]
        tick_labels = frame_ids[::tick_step]

        # Translation residual plot.
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.bar(positions, translation_residuals_mm)
        ax.axhline(
            translation_threshold_mm,
            linestyle="--",
            label="MAD threshold = {:.3f} mm".format(
                translation_threshold_mm
            ),
        )
        if np.any(outlier_mask):
            ax.scatter(
                positions[outlier_mask],
                translation_residuals_mm[outlier_mask],
                marker="x",
                s=70,
                label="Suspected outlier",
                zorder=5,
            )
        ax.set_xticks(tick_positions)
        ax.set_xticklabels(tick_labels, rotation=45)
        ax.set_xlabel("Frame ID")
        ax.set_ylabel("Translation Residual (mm)")
        ax.set_title(
            "Fixed-Target Translation Residual - {}".format(method_name)
        )
        ax.legend()
        fig.tight_layout()

        translation_plot_path = os.path.join(
            self.final_transforms_dir,
            "FixedTargetTranslationResidual_{}.png".format(method_name),
        )
        fig.savefig(
            translation_plot_path,
            dpi=200,
            bbox_inches="tight",
        )
        plt.close(fig)

        # Rotation residual plot.
        fig, ax = plt.subplots(figsize=(12, 6))
        ax.bar(positions, rotation_residuals_deg)
        ax.axhline(
            rotation_threshold_deg,
            linestyle="--",
            label="MAD threshold = {:.4f} deg".format(
                rotation_threshold_deg
            ),
        )
        if np.any(outlier_mask):
            ax.scatter(
                positions[outlier_mask],
                rotation_residuals_deg[outlier_mask],
                marker="x",
                s=70,
                label="Suspected outlier",
                zorder=5,
            )
        ax.set_xticks(tick_positions)
        ax.set_xticklabels(tick_labels, rotation=45)
        ax.set_xlabel("Frame ID")
        ax.set_ylabel("Rotation Residual (deg)")
        ax.set_title(
            "Fixed-Target Rotation Residual - {}".format(method_name)
        )
        ax.legend()
        fig.tight_layout()

        rotation_plot_path = os.path.join(
            self.final_transforms_dir,
            "FixedTargetRotationResidual_{}.png".format(method_name),
        )
        fig.savefig(
            rotation_plot_path,
            dpi=200,
            bbox_inches="tight",
        )
        plt.close(fig)

        print(
            "Saved translation residual plot:",
            translation_plot_path,
        )
        print(
            "Saved rotation residual plot:",
            rotation_plot_path,
        )

    def run_robot_world_hand_eye_calibration(self):
        """Optional robot-world/hand-eye calibration using corrected inputs."""
        if not hasattr(cv2, "calibrateRobotWorldHandEye"):
            print(
                "cv2.calibrateRobotWorldHandEye is unavailable in "
                "this OpenCV version; skipping it."
            )
            return

        # MODIFIED: calibrateRobotWorldHandEye requires base2gripper, which is
        # the inverse of the Excel-derived gripper2base pose.
        T_base2gripper_list = [
            np.linalg.inv(T)
            for T in self.T_gripper2base_list
        ]
        R_base2gripper = [
            T[:3, :3]
            for T in T_base2gripper_list
        ]
        t_base2gripper = [
            T[:3, 3].reshape(3, 1)
            for T in T_base2gripper_list
        ]

        methods = [
            ("SHAH", cv2.CALIB_ROBOT_WORLD_HAND_EYE_SHAH),
            ("LI", cv2.CALIB_ROBOT_WORLD_HAND_EYE_LI),
        ]

        print("\nRunning cv2.calibrateRobotWorldHandEye...")
        for method_name, method_flag in methods:
            (
                R_base2world,
                t_base2world,
                R_gripper2cam,
                t_gripper2cam,
            ) = cv2.calibrateRobotWorldHandEye(
                self.RTarget2Cam,
                self.TTarget2Cam,
                R_base2gripper,
                t_base2gripper,
                method=method_flag,
            )

            T_base2world = self.make_transform(
                R_base2world,
                t_base2world,
            )
            T_gripper2cam = self.make_transform(
                R_gripper2cam,
                t_gripper2cam,
            )
            T_cam2gripper = np.linalg.inv(T_gripper2cam)

            print("\nRobot-world/hand-eye method:", method_name)
            print("T_base2world:\n", T_base2world)
            print("T_gripper2cam:\n", T_gripper2cam)

            np.savez(
                os.path.join(
                    self.final_transforms_dir,
                    "RobotWorld_T_base2world_{}.npz".format(method_name),
                ),
                T=T_base2world,
                method=np.array(method_name),
                valid_frame_ids=np.asarray(self.valid_frame_ids),
            )
            np.savez(
                os.path.join(
                    self.final_transforms_dir,
                    "RobotWorld_T_gripper2cam_{}.npz".format(method_name),
                ),
                T=T_gripper2cam,
                method=np.array(method_name),
                valid_frame_ids=np.asarray(self.valid_frame_ids),
            )
            np.savez(
                os.path.join(
                    self.final_transforms_dir,
                    "RobotWorld_T_cam2gripper_{}.npz".format(method_name),
                ),
                T=T_cam2gripper,
                method=np.array(method_name),
                valid_frame_ids=np.asarray(self.valid_frame_ids),
            )


if __name__ == "__main__":
    repository_root = Path(__file__).resolve().parents[1]
    image_folder = repository_root / "data/raw/images"
    pose_folder = (
        repository_root
        / "calibration/handeye_gemini/robot_poses"
    )
    output_root = repository_root / "calibration/handeye_gemini"
    camera_info_path = repository_root / "data/raw/camera_info.json"

    with camera_info_path.open("r", encoding="utf-8") as stream:
        camera_info = json.load(stream)
    color_camera = camera_info["color_camera"]
    if (
        color_camera["intrinsic"]["width"],
        color_camera["intrinsic"]["height"],
    ) != (1280, 720):
        raise ValueError("Gemini RGB resolution is not 1280x720")
    K = np.asarray(
        color_camera["intrinsic"]["camera_matrix"], dtype=np.float64
    )
    D = np.asarray(
        color_camera["distortion"]["opencv_order"], dtype=np.float64
    )
    expected_valid_frame_ids = [
        frame_id for frame_id in range(1, 76) if frame_id != 42
    ]

    calibration = CameraCalibration(
        image_folder=str(image_folder),
        transforms_folder=str(pose_folder),
        pattern_size=(11, 8),
        square_size=0.015,
        ShowProjectError=False,
        ShowCorners=False,
        RunRobotWorldHandEye=False,
        intrinsic_matrix=K,
        distortion_coefficients=D,
        output_root=str(output_root),
        expected_valid_frame_ids=expected_valid_frame_ids,
    )
