
import os
import json
import copy
import gc
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import open3d as o3d
import cv2
import matplotlib.pyplot as plt

from scipy.spatial import cKDTree

from sklearn.preprocessing import StandardScaler
from sklearn.cluster import KMeans, MiniBatchKMeans
from sklearn.decomposition import PCA

from sklearn.metrics import (
    silhouette_score,
    calinski_harabasz_score,
    davies_bouldin_score
)


# ============================================================
# 1. Arguments
# ============================================================

parser = argparse.ArgumentParser()

parser.add_argument(
    "--input",
    default="/home/root01/ramanseg/data/processed/roi/"
            "ramanseg_stomach_roi_coarse.ply"
)

parser.add_argument(
    "--output",
    default="/home/root01/ramanseg/data/processed/"
            "unsupervised_stage1"
)

parser.add_argument(
    "--report",
    default="/home/root01/ramanseg/reports/"
            "stage1_unsupervised"
)

parser.add_argument("--k-neighbors", type=int, default=30)
parser.add_argument("--chunk-size", type=int, default=10000)

args = parser.parse_args()

INPUT = Path(args.input)
OUTPUT = Path(args.output)
REPORT = Path(args.report)

for d in [
    OUTPUT / "ply",
    OUTPUT / "features",
    OUTPUT / "labels",
    OUTPUT / "plots",
    REPORT
]:
    d.mkdir(parents=True, exist_ok=True)


# ============================================================
# 2. Read point cloud
# ============================================================

print("\n========== LOAD POINT CLOUD ==========")

pcd = o3d.io.read_point_cloud(str(INPUT))

points = np.asarray(pcd.points).astype(np.float32)
rgb = np.asarray(pcd.colors).astype(np.float32)

if len(points) == 0:
    raise RuntimeError("Empty point cloud")

if not pcd.has_colors():
    raise RuntimeError("Input PLY has no RGB colors")

if not np.isfinite(points).all():
    raise RuntimeError("Non-finite XYZ values detected")

if not np.isfinite(rgb).all():
    raise RuntimeError("Non-finite RGB values detected")

N = len(points)

if N <= args.k_neighbors:
    raise RuntimeError("Not enough points for KNN")

print("Input:", INPUT)
print("Points:", N)
print("RGB:", rgb.shape)
print("XYZ range:", points.max(axis=0) - points.min(axis=0))


# ============================================================
# 3. Color feature extraction
# ============================================================

print("\n========== COLOR FEATURES ==========")

rgb = np.clip(rgb, 0.0, 1.0)

# RGB
rgb_features = rgb.copy()

# HSV
rgb_u8 = np.round(rgb * 255).astype(np.uint8)

hsv_cv = cv2.cvtColor(
    rgb_u8.reshape(-1, 1, 3),
    cv2.COLOR_RGB2HSV
).reshape(-1, 3)

h = hsv_cv[:, 0].astype(np.float32)
s = hsv_cv[:, 1].astype(np.float32) / 255.0
v = hsv_cv[:, 2].astype(np.float32) / 255.0

theta = h * (2.0 * np.pi / 180.0)

hsv_features = np.column_stack([
    np.sin(theta),
    np.cos(theta),
    s,
    v
]).astype(np.float32)

# CIELAB
lab_features = cv2.cvtColor(
    rgb.reshape(-1, 1, 3),
    cv2.COLOR_RGB2LAB
).reshape(-1, 3).astype(np.float32)

print("RGB:", rgb_features.shape)
print("HSV:", hsv_features.shape)
print("Lab:", lab_features.shape)


# ============================================================
# 4. Geometry feature extraction
# ============================================================

def extract_geometry(points, k=30, chunk_size=10000):

    print("\n========== GEOMETRY EXTRACTION ==========")
    print("KNN:", k)

    n = len(points)

    tree = cKDTree(points)

    geometry = np.zeros((n, 6), dtype=np.float32)

    normals = np.zeros((n, 3), dtype=np.float32)

    eps = 1e-12

    # --------------------------------------------------------
    # First pass: local PCA
    # --------------------------------------------------------

    for start in range(0, n, chunk_size):

        end = min(start + chunk_size, n)

        _, idx = tree.query(
            points[start:end],
            k=k,
            workers=-1
        )

        neighbors = points[idx]

        centered = (
            neighbors -
            neighbors.mean(axis=1, keepdims=True)
        )

        covariance = np.einsum(
            "bki,bkj->bij",
            centered,
            centered,
            optimize=True
        ) / k

        eigenvalues, eigenvectors = np.linalg.eigh(
            covariance
        )

        # eigh returns ascending eigenvalues

        l3 = np.maximum(eigenvalues[:, 0], 0.0)
        l2 = np.maximum(eigenvalues[:, 1], 0.0)
        l1 = np.maximum(eigenvalues[:, 2], 0.0)

        normals[start:end] = eigenvectors[:, :, 0]

        # Curvature
        curvature = l3 / (l1 + l2 + l3 + eps)

        # Roughness
        roughness = np.sqrt(l3)

        # Planarity
        planarity = (l2 - l3) / (l1 + eps)

        # Linearity
        linearity = (l1 - l2) / (l1 + eps)

        # Scattering
        scattering = l3 / (l1 + eps)

        geometry[start:end, 0] = curvature
        geometry[start:end, 1] = roughness
        geometry[start:end, 2] = planarity
        geometry[start:end, 3] = linearity
        geometry[start:end, 4] = scattering

        print(
            "PCA:",
            end,
            "/",
            n
        )

    # --------------------------------------------------------
    # Second pass: normal variation
    # --------------------------------------------------------

    print("\nCalculating normal variation...")

    for start in range(0, n, chunk_size):

        end = min(start + chunk_size, n)

        _, idx = tree.query(
            points[start:end],
            k=k,
            workers=-1
        )

        neighbor_normals = normals[idx]

        center_normals = normals[start:end]

        dots = np.abs(
            np.sum(
                neighbor_normals *
                center_normals[:, None, :],
                axis=2
            )
        )

        variation = np.mean(
            1.0 - np.clip(dots, 0.0, 1.0),
            axis=1
        )

        geometry[start:end, 5] = variation

        print(
            "Normal variation:",
            end,
            "/",
            n
        )

    del tree, normals

    return geometry


geometry_names = [
    "curvature",
    "roughness_m",
    "planarity",
    "linearity",
    "scattering",
    "normal_variation"
]

geometry_features = extract_geometry(
    points,
    k=args.k_neighbors,
    chunk_size=args.chunk_size
)

print("Geometry shape:", geometry_features.shape)


# ============================================================
# 5. Save raw point features
# ============================================================

print("\n========== SAVE FEATURES ==========")

np.savez_compressed(
    OUTPUT / "features" / "point_features.npz",

    xyz=points,
    rgb=rgb_features,
    hsv=hsv_features,
    lab=lab_features,
    geometry=geometry_features
)

print("Saved point_features.npz")


# ============================================================
# 6. Feature configurations
# ============================================================

def zscore(x):

    return StandardScaler().fit_transform(
        x
    ).astype(np.float32)


def stratified_sample(labels, max_samples=3000):

    rng = np.random.default_rng(42)

    classes = np.unique(labels)

    per_class = max(1, max_samples // len(classes))

    selected = []

    for c in classes:

        idx = np.flatnonzero(labels == c)

        take = min(len(idx), per_class)

        selected.append(
            rng.choice(idx, size=take, replace=False)
        )

    return np.concatenate(selected)


feature_configs = [

    (
        "rgb",
        rgb_features,
        ["R", "G", "B"]
    ),

    (
        "hsv",
        hsv_features,
        ["H_sin", "H_cos", "S", "V"]
    ),

    (
        "lab",
        lab_features,
        ["L", "a", "b"]
    ),

    (
        "geometry",
        geometry_features,
        geometry_names
    ),

    (
        "lab_geometry",
        np.column_stack([
            lab_features,
            geometry_features
        ]).astype(np.float32),

        ["L", "a", "b"] + geometry_names
    )
]


# ============================================================
# 7. Clustering
# ============================================================

palette = np.array([
    [0.95, 0.12, 0.12],
    [0.10, 0.35, 0.95],
    [0.10, 0.75, 0.20]
], dtype=np.float64)

all_labels = {}

metrics_rows = []

statistics_rows = []

rng = np.random.default_rng(42)

for name, raw_features, names in feature_configs:

    print("\n" + "=" * 65)
    print("FEATURE SET:", name)
    print("=" * 65)

    # --------------------------------------------------------
    # Feature normalization
    # --------------------------------------------------------

    if name == "lab_geometry":

        # Equalize the total contribution of color
        # and geometry feature blocks.

        color_z = zscore(lab_features)
        geometry_z = zscore(geometry_features)

        X = np.column_stack([
            color_z / np.sqrt(color_z.shape[1]),
            geometry_z / np.sqrt(geometry_z.shape[1])
        ]).astype(np.float32)

    else:

        X = zscore(raw_features)

    # --------------------------------------------------------
    # K = 2 and K = 3
    # --------------------------------------------------------

    for k in [2, 3]:

        print("\nRunning KMeans, K =", k)

        if N > 300000:

            model = MiniBatchKMeans(
                n_clusters=k,
                random_state=42,
                n_init=10,
                batch_size=8192,
                max_iter=200
            )

            algorithm = "MiniBatchKMeans"

        else:

            model = KMeans(
                n_clusters=k,
                random_state=42,
                n_init=10,
                max_iter=300
            )

            algorithm = "KMeans"

        labels = model.fit_predict(X)

        labels = labels.astype(np.uint8)

        key = "{}_k{}".format(name, k)

        all_labels[key] = labels

        # ----------------------------------------------------
        # Save cluster-colored PLY
        # ----------------------------------------------------

        colors = palette[labels]

        output_pcd = copy.deepcopy(pcd)

        output_pcd.colors = o3d.utility.Vector3dVector(
            colors
        )

        ply_path = (
            OUTPUT / "ply" /
            "{}.ply".format(key)
        )

        o3d.io.write_point_cloud(
            str(ply_path),
            output_pcd,
            write_ascii=False,
            compressed=True
        )

        print("Saved:", ply_path)

        # ----------------------------------------------------
        # Save labels
        # ----------------------------------------------------

        np.save(
            OUTPUT / "labels" /
            "{}.npy".format(key),
            labels
        )

        # ----------------------------------------------------
        # Cluster statistics
        # ----------------------------------------------------

        unique, counts = np.unique(
            labels,
            return_counts=True
        )

        for c, count in zip(unique, counts):

            mask = labels == c

            row = {
                "feature_set": name,
                "K": k,
                "cluster": int(c),
                "point_count": int(count),
                "ratio": float(count / N)
            }

            means = raw_features[mask].mean(axis=0)

            for feature_name, value in zip(names, means):

                row["mean_" + feature_name] = float(value)

            statistics_rows.append(row)

        # ----------------------------------------------------
        # Internal clustering metrics
        # ----------------------------------------------------

        sample_idx = stratified_sample(labels)

        X_sample = X[sample_idx]
        y_sample = labels[sample_idx]

        metric_values = {
            "silhouette": np.nan,
            "calinski_harabasz": np.nan,
            "davies_bouldin": np.nan
        }

        if len(np.unique(y_sample)) > 1:

            try:
                metric_values["silhouette"] = float(
                    silhouette_score(
                        X_sample,
                        y_sample
                    )
                )

                metric_values["calinski_harabasz"] = float(
                    calinski_harabasz_score(
                        X_sample,
                        y_sample
                    )
                )

                metric_values["davies_bouldin"] = float(
                    davies_bouldin_score(
                        X_sample,
                        y_sample
                    )
                )

            except Exception as e:

                print("Metric warning:", e)

        metrics_rows.append({

            "feature_set": name,
            "K": k,
            "algorithm": algorithm,
            "point_count": N,
            "actual_clusters": len(unique),

            **metric_values

        })

        # ----------------------------------------------------
        # PCA visualization
        # ----------------------------------------------------

        pca_idx = rng.choice(
            N,
            size=min(30000, N),
            replace=False
        )

        pca = PCA(n_components=2)

        pca_result = pca.fit_transform(
            X[pca_idx]
        )

        plt.figure(figsize=(9, 7))

        for c in unique:

            mask = labels[pca_idx] == c

            plt.scatter(
                pca_result[mask, 0],
                pca_result[mask, 1],
                s=2,
                alpha=0.5,
                color=palette[c],
                label="Cluster {}".format(c)
            )

        plt.title(
            "{} / K={}".format(name, k)
        )

        plt.xlabel("PC1")
        plt.ylabel("PC2")

        plt.legend(markerscale=4)

        plt.tight_layout()

        plot_path = (
            OUTPUT / "plots" /
            "{}_pca.png".format(key)
        )

        plt.savefig(
            plot_path,
            dpi=200
        )

        plt.close()

        print("Saved PCA:", plot_path)

    del X
    gc.collect()


# ============================================================
# 8. Save all labels and statistics
# ============================================================

print("\n========== SAVE RESULTS ==========")

np.savez_compressed(
    OUTPUT / "labels" / "cluster_labels.npz",
    **all_labels
)

pd.DataFrame(
    metrics_rows
).to_csv(
    REPORT / "clustering_metrics.csv",
    index=False
)

pd.DataFrame(
    statistics_rows
).to_csv(
    REPORT / "cluster_statistics.csv",
    index=False
)


# ------------------------------------------------------------
# Point feature sample CSV
# ------------------------------------------------------------

sample_idx = rng.choice(
    N,
    size=min(50000, N),
    replace=False
)

sample_data = {

    "point_index": sample_idx,

    "x": points[sample_idx, 0],
    "y": points[sample_idx, 1],
    "z": points[sample_idx, 2],

    "R": rgb_features[sample_idx, 0],
    "G": rgb_features[sample_idx, 1],
    "B": rgb_features[sample_idx, 2],

    "Lab_L": lab_features[sample_idx, 0],
    "Lab_a": lab_features[sample_idx, 1],
    "Lab_b": lab_features[sample_idx, 2]
}

for j, feature_name in enumerate(geometry_names):

    sample_data[feature_name] = geometry_features[
        sample_idx, j
    ]

for key, labels in all_labels.items():

    sample_data["cluster_" + key] = labels[sample_idx]

pd.DataFrame(
    sample_data
).to_csv(
    OUTPUT / "features" /
    "point_features_sample.csv",
    index=False
)


# ============================================================
# 9. Save experiment metadata
# ============================================================

info = {

    "input": str(INPUT),

    "point_count": int(N),

    "knn": args.k_neighbors,

    "chunk_size": args.chunk_size,

    "feature_sets": [
        "rgb",
        "hsv",
        "lab",
        "geometry",
        "lab_geometry"
    ],

    "cluster_numbers": [2, 3],

    "manual_labels": False,

    "manual_segmentation": False,

    "spatial_coordinates_used_for_clustering": False,

    "outlier_removal": False,

    "random_state": 42

}

with open(
    REPORT / "experiment_info.json",
    "w",
    encoding="utf-8"
) as f:

    json.dump(
        info,
        f,
        indent=4,
        ensure_ascii=False
    )


print("\n" + "=" * 65)
print("STAGE 1 COMPLETED")
print("=" * 65)

print("Input points:", N)

print("PLY output:", OUTPUT / "ply")

print("Features:", OUTPUT / "features")

print("Labels:", OUTPUT / "labels")

print("Plots:", OUTPUT / "plots")

print("Reports:", REPORT)

print("=" * 65)
