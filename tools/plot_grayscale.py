import os
import glob
import cv2
import numpy as np
from matplotlib import pyplot as plt

folder = "/home/zukimo/XMC_ROS2/calib_data/realsense_pico_charuco_20250919_3/tof_gray"
pattern = os.path.join(folder, "gray_*")

# folder = "/home/zukimo/XMC_ROS2/calib_data/realsense_pico_charuco_20250919_3/calib/tof/undistorted_preview"
# pattern = os.path.join(folder, 'ud_*')

files = sorted(glob.glob(pattern))
if not files:
    raise FileNotFoundError(f"No files found matching {pattern}")

print(f"Found {len(files)} files")

for i, f in enumerate(files, 1):
    img = cv2.imread(f, cv2.IMREAD_UNCHANGED)
    if img is None:
        print(f"Could not read {f}")
        continue

    print(f"[{i}/{len(files)}] File: {f}")
    print("   Shape:", img.shape, " Dtype:", img.dtype)

    # Normalize for display if 16-bit
    to_show = img
    if img.dtype == np.uint16:
        max_val = img.max() if img.max() > 0 else 1
        to_show = (img.astype(np.float32) / max_val * 255.0).astype(np.uint8)

    plt.figure()
    plt.imshow(to_show, cmap="gray")
    plt.title(f"Depth Image {i}")
    plt.axis("off")
    plt.show()
