import cv2
import csv
import re
from pathlib import Path

# =====================================================
# Configuration
# =====================================================

IMAGE_DIR = Path("data/raw/images")

OUTPUT_DIR = Path(
    "calibration/handeye_gemini/diagnostics/corners"
)

PATTERN_SIZE = (11, 8)
EXPECTED_CORNERS = 11 * 8

OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

# =====================================================
# Find images
# =====================================================

images = sorted(
    list(IMAGE_DIR.glob("*.png")) +
    list(IMAGE_DIR.glob("*.jpg")) +
    list(IMAGE_DIR.glob("*.jpeg"))
)

print("=" * 65)
print("GEMINI 336L CHECKERBOARD DETECTION")
print("=" * 65)

print("Image directory:", IMAGE_DIR)
print("Total images:", len(images))
print("Pattern size:", PATTERN_SIZE)
print("Expected corners:", EXPECTED_CORNERS)

if not images:
    raise RuntimeError("No RGB images found.")

results = []

# =====================================================
# Detection
# =====================================================

for image_path in images:

    image = cv2.imread(str(image_path))

    if image is None:
        print("READ FAILED:", image_path.name)
        continue

    gray = cv2.cvtColor(
        image,
        cv2.COLOR_BGR2GRAY
    )

    height, width = gray.shape

    found = False
    corners = None
    detector = "NONE"

    # First: findChessboardCornersSB
    if hasattr(cv2, "findChessboardCornersSB"):

        flags = (
            cv2.CALIB_CB_NORMALIZE_IMAGE |
            cv2.CALIB_CB_EXHAUSTIVE |
            cv2.CALIB_CB_ACCURACY
        )

        found, corners = cv2.findChessboardCornersSB(
            gray,
            PATTERN_SIZE,
            flags=flags
        )

        if found:
            detector = "SB"

    # Fallback: traditional detector
    if not found:

        flags = (
            cv2.CALIB_CB_ADAPTIVE_THRESH |
            cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        found, corners = cv2.findChessboardCorners(
            gray,
            PATTERN_SIZE,
            flags
        )

        if found:

            criteria = (
                cv2.TERM_CRITERIA_EPS +
                cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.001
            )

            corners = cv2.cornerSubPix(
                gray,
                corners,
                (5, 5),
                (-1, -1),
                criteria
            )

            detector = "CLASSIC"

    # =================================================
    # Visualization
    # =================================================

    vis = image.copy()

    frame_match = re.search(
        r"(\d+)",
        image_path.stem
    )

    frame_id = (
        int(frame_match.group(1))
        if frame_match else -1
    )

    if found:

        count = len(corners)

        cv2.drawChessboardCorners(
            vis,
            PATTERN_SIZE,
            corners,
            True
        )

        status = "DETECTED"

    else:

        count = 0
        status = "FAILED"

        cv2.putText(
            vis,
            "NOT DETECTED",
            (30, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 0, 255),
            2
        )

    output_path = (
        OUTPUT_DIR /
        f"{image_path.stem}_corners.png"
    )

    cv2.imwrite(str(output_path), vis)

    results.append([
        frame_id,
        image_path.name,
        width,
        height,
        detector,
        status,
        count
    ])

    print(
        f"Frame {frame_id:03d}: "
        f"{status:10s} "
        f"corners={count:2d} "
        f"detector={detector}"
    )

# =====================================================
# Save CSV
# =====================================================

csv_path = OUTPUT_DIR / "corner_detection_results.csv"

with open(
    csv_path,
    "w",
    newline="",
    encoding="utf-8"
) as f:

    writer = csv.writer(f)

    writer.writerow([
        "frame_id",
        "filename",
        "width",
        "height",
        "detector",
        "status",
        "corner_count"
    ])

    writer.writerows(results)

# =====================================================
# Summary
# =====================================================

success = [
    x for x in results
    if x[5] == "DETECTED"
]

failed = [
    x for x in results
    if x[5] == "FAILED"
]

print("\n" + "=" * 65)
print("DETECTION SUMMARY")
print("=" * 65)

print("Total images:", len(results))
print("Detected:", len(success))
print("Failed:", len(failed))

print(
    "Detection rate:",
    f"{100 * len(success) / max(len(results), 1):.2f}%"
)

print("\nSuccessful frame IDs:")
print([x[0] for x in success])

print("\nFailed frame IDs:")
print([x[0] for x in failed])

print("\nCSV:", csv_path)
print("Visualizations:", OUTPUT_DIR)
