
import argparse
from pathlib import Path

import numpy as np
import pandas as pd
import open3d as o3d

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
from matplotlib.patches import Patch


# ============================================================
# 1. Paths
# ============================================================

ROOT = Path("/home/root01/ramanseg")

ORIGINAL = (
    ROOT / "data/processed/roi/"
    "ramanseg_stomach_roi_coarse.ply"
)

RESULT_DIR = (
    ROOT / "data/processed/unsupervised_stage1/ply"
)

OUTPUT_DIR = (
    ROOT / "reports/stage1_unsupervised/side_by_side"
)


# ============================================================
# 2. Camera configuration
# ============================================================

# Z-up: use world Z as the vertical axis.
#
# azim controls the horizontal viewing direction.
# elev controls camera elevation.
#
# All comparison images share these parameters.

VIEW_ELEV = 90
VIEW_AZIM = -90

# Orthographic projection
ORTHOGRAPHIC = True

# Maximum number of points used for rendering.
# This only affects visualization, NOT source PLY files.

MAX_POINTS = 120000

RANDOM_SEED = 42

# Point size
POINT_SIZE = 3.0


# ============================================================
# 3. Cluster palette
# ============================================================

PALETTE = np.array([
    [0.95, 0.12, 0.12],  # Cluster 0: red
    [0.10, 0.35, 0.95],  # Cluster 1: blue
    [0.10, 0.75, 0.20]   # Cluster 2: green
])


# ============================================================
# 4. Read point cloud
# ============================================================

def load_cloud(path):

    pcd = o3d.io.read_point_cloud(str(path))

    if len(pcd.points) == 0:
        raise RuntimeError(f"Empty point cloud: {path}")

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)

    if not pcd.has_colors():
        raise RuntimeError(f"No RGB colors: {path}")

    return points, colors


# ============================================================
# 5. Verify correspondence
# ============================================================

def verify_clouds(original, result, result_path):

    if len(original) != len(result):

        raise RuntimeError(
            f"Point count mismatch: {result_path}\n"
            f"Original: {len(original)}\n"
            f"Result: {len(result)}"
        )

    max_diff = np.max(
        np.abs(original - result)
    )

    if max_diff > 1e-5:

        raise RuntimeError(
            f"XYZ correspondence check failed: {result_path}\n"
            f"Maximum coordinate difference: {max_diff}"
        )

    return max_diff


# ============================================================
# 6. Camera limits
# ============================================================

def calculate_bounds(points):

    xyz_min = points.min(axis=0)
    xyz_max = points.max(axis=0)

    center = (xyz_min + xyz_max) / 2.0

    ranges = xyz_max - xyz_min

    margin = ranges * 0.04

    lower = xyz_min - margin
    upper = xyz_max + margin

    ranges = upper - lower

    return lower, upper, center, ranges


# ============================================================
# 7. Draw one panel
# ============================================================

def draw_panel(
    ax,
    points,
    colors,
    title,
    lower,
    upper,
    ranges,
    cluster_legend=False
):

    ax.scatter(
        points[:, 0],
        points[:, 1],
        points[:, 2],

        c=colors,

        s=POINT_SIZE,

        marker=".",

        linewidths=0,

        alpha=0.95,

        depthshade=False,

        rasterized=True
    )

    ax.set_title(
        title,
        fontsize=13,
        pad=12
    )

    ax.set_xlim(lower[0], upper[0])
    ax.set_ylim(lower[1], upper[1])
    ax.set_zlim(lower[2], upper[2])

    # Equal metric scale
    ax.set_box_aspect(ranges)

    # Z is the vertical axis
    ax.view_init(
        elev=VIEW_ELEV,
        azim=VIEW_AZIM
    )

    if ORTHOGRAPHIC:
        ax.set_proj_type("ortho")

    ax.set_xlabel("X (m)", fontsize=9)
    ax.set_ylabel("Y (m)", fontsize=9)

    if abs(VIEW_ELEV) == 90:
        ax.set_zlabel("")
        ax.set_zticks([])
    else:
        ax.set_zlabel("Z (m)", fontsize=9)

    ax.tick_params(
        axis="both",
        which="major",
        labelsize=7
    )

    ax.grid(False)

    for axis in [
        ax.xaxis,
        ax.yaxis,
        ax.zaxis
    ]:
        axis.pane.set_facecolor(
            (1, 1, 1, 0)
        )

    if cluster_legend:

        cluster_ids = np.argmin(
            np.sum(
                (colors[:, None, :] - PALETTE[None, :, :]) ** 2,
                axis=2
            ),
            axis=1
        )

        handles = []

        for cid in range(3):

            count = np.sum(cluster_ids == cid)

            ratio = count / len(cluster_ids)

            handles.append(
                Patch(
                    facecolor=PALETTE[cid],
                    label=f"Cluster {cid}: {ratio:.1%}"
                )
            )

        ax.legend(
            handles=handles,
            loc="upper right",
            fontsize=8,
            framealpha=0.9
        )


# ============================================================
# 8. Render side-by-side comparison
# ============================================================

def render_comparison(
    original_points,
    original_colors,
    result_points,
    result_colors,
    result_path,
    sample_indices,
    bounds
):

    lower, upper, center, ranges = bounds

    # Same point indices for both panels
    xyz_original = original_points[sample_indices]
    rgb_original = original_colors[sample_indices]

    xyz_result = result_points[sample_indices]
    rgb_result = result_colors[sample_indices]

    fig = plt.figure(
        figsize=(17, 8),
        dpi=180,
        facecolor="white"
    )

    ax1 = fig.add_subplot(
        1, 2, 1,
        projection="3d"
    )

    ax2 = fig.add_subplot(
        1, 2, 2,
        projection="3d"
    )

    draw_panel(
        ax1,
        xyz_original,
        rgb_original,
        "Original RGB Point Cloud",
        lower,
        upper,
        ranges
    )

    feature_name = result_path.stem.replace("_k3", "")

    draw_panel(
        ax2,
        xyz_result,
        rgb_result,
        f"Unsupervised Clustering - {feature_name.upper()} (K=3)",
        lower,
        upper,
        ranges,
        cluster_legend=True
    )

    fig.suptitle(
        "RamanSeg: Original vs Unsupervised Point Cloud",
        fontsize=16,
        fontweight="bold",
        y=0.97
    )

    plt.subplots_adjust(
        left=0.02,
        right=0.98,
        top=0.90,
        bottom=0.06,
        wspace=0.02
    )

    output_path = (
        OUTPUT_DIR /
        f"compare_{feature_name}_k3.png"
    )

    fig.savefig(
        output_path,
        dpi=220,
        bbox_inches="tight",
        facecolor="white"
    )

    plt.close(fig)

    print("Saved:", output_path)

    return output_path


# ============================================================
# 9. Main batch processing
# ============================================================

def main():

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True
    )

    print("=" * 65)
    print("RAMANSEG K=3 BATCH COMPARISON")
    print("=" * 65)

    print("\nLoading original point cloud...")

    original_points, original_colors = load_cloud(
        ORIGINAL
    )

    N = len(original_points)

    print("Original points:", N)

    print("\nCalculating common camera bounds...")

    bounds = calculate_bounds(
        original_points
    )

    lower, upper, center, ranges = bounds

    print("XYZ center:", center)
    print("XYZ range:", ranges)

    # --------------------------------------------------------
    # Deterministic common sampling
    # --------------------------------------------------------

    rng = np.random.default_rng(
        RANDOM_SEED
    )

    sample_count = min(
        N,
        MAX_POINTS
    )

    if sample_count < N:

        sample_indices = rng.choice(
            N,
            size=sample_count,
            replace=False
        )

        sample_indices.sort()

    else:

        sample_indices = np.arange(N)

    print("Rendering points:", sample_count)

    # --------------------------------------------------------
    # Find all K=3 results
    # --------------------------------------------------------

    preferred_order = [
        "rgb_k3.ply",
        "hsv_k3.ply",
        "lab_k3.ply",
        "geometry_k3.ply",
        "lab_geometry_k3.ply"
    ]

    result_files = [
        RESULT_DIR / name
        for name in preferred_order
        if (RESULT_DIR / name).exists()
    ]

    if not result_files:

        raise FileNotFoundError(
            f"No K=3 PLY files found in {RESULT_DIR}"
        )

    print("\nK=3 files found:", len(result_files))

    manifest = []

    # --------------------------------------------------------
    # Batch rendering
    # --------------------------------------------------------

    for i, result_path in enumerate(result_files, 1):

        print("\n" + "-" * 60)

        print(
            f"[{i}/{len(result_files)}] "
            f"Processing {result_path.name}"
        )

        result_points, result_colors = load_cloud(
            result_path
        )

        max_diff = verify_clouds(
            original_points,
            result_points,
            result_path
        )

        print("XYZ correspondence: PASS")
        print("Max XYZ difference:", max_diff)

        output_path = render_comparison(
            original_points,
            original_colors,
            result_points,
            result_colors,
            result_path,
            sample_indices,
            bounds
        )

        manifest.append({

            "result": result_path.name,

            "original_points": N,

            "rendered_points": sample_count,

            "max_xyz_difference": max_diff,

            "view_elevation": VIEW_ELEV,

            "view_azimuth": VIEW_AZIM,

            "output": str(output_path)

        })

    # --------------------------------------------------------
    # Save manifest
    # --------------------------------------------------------

    pd.DataFrame(
        manifest
    ).to_csv(
        OUTPUT_DIR / "comparison_manifest.csv",
        index=False
    )

    print("\n" + "=" * 65)
    print("BATCH COMPARISON COMPLETED")
    print("=" * 65)

    print("Original:", ORIGINAL)

    print("Results:", RESULT_DIR)

    print("Output:", OUTPUT_DIR)

    print("Total comparisons:", len(manifest))

    print("=" * 65)


if __name__ == "__main__":
    main()
