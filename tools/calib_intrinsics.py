#!/usr/bin/env python3
"""
Calibrate intrinsics for two sensors from a recorded dataset:
- D415 RGB images:        <dataset>/rgb/rgb_*.png
- Pico Flexx amplitude:   <dataset>/tof_amp/amp_*.png   (grayscale)

Outputs JSON files with K, D and reprojection error, plus optional undistorted previews.

Usage:
  python3 calib_intrinsics.py --dataset /path/to/calib_data/session_x \
      --rgb_square 0.030 --rgb_cols 6 --rgb_rows 9 \
      --tof_square 0.030 --tof_cols 6 --tof_rows 9 \
      --charuco 1 --aruco_dict 4 --save_undistorted 1

Notes:
- You can calibrate plain checkerboard or ChArUco. For ChArUco, pass --charuco 1 and pick an ArUco dictionary.
- For checkerboard: cols = inner corners across, rows = inner corners down.
- For ChArUco: cols/rows are the number of chessboard squares (NOT inner corners), square size is in meters.
- If your amplitude images are 16-bit, the script auto-normalizes for corner detection only; calibration uses the original geometry.
"""

import os
import glob
import json
import argparse
from typing import List, Tuple

import numpy as np
import cv2


def find_checker_corners(gray, cols, rows):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not ok:
        return False, None
    # refine
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
    return True, corners

def find_checker_corners_2(gray, cols, rows):
    # Prefer the SB detector (OpenCV >= 4.5)
    flags = (cv2.CALIB_CB_EXHAUSTIVE |
             cv2.CALIB_CB_NORMALIZE_IMAGE |
             cv2.CALIB_CB_ACCURACY)
    ok, corners = cv2.findChessboardCornersSB(gray, (cols, rows), flags)
    if ok:
        return True, corners.reshape(-1,1,2).astype(np.float32)

    # Fallback: legacy detector with subpixel refine
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not ok:
        return False, None
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-3)
    cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), term)
    return True, corners


def create_board_object_points(cols, rows, square):
    # for checkerboard: cols x rows inner corners
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square)
    return objp


def detect_charuco(gray, board, aruco_dict):
    params = cv2.aruco.DetectorParameters_create()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

    # Make the detector accept small, blurry markers
    params.minMarkerPerimeterRate = 0.01     # lower than 0.03 default
    params.maxMarkerPerimeterRate = 4.0
    params.minCornerDistanceRate = 0.02
    params.minDistanceToBorder = 2

    # Try a broader adaptive threshold sweep
    params.adaptiveThreshWinSizeMin = 5
    params.adaptiveThreshWinSizeMax = 45
    params.adaptiveThreshWinSizeStep = 5
    params.adaptiveThreshConstant = 7       # small bias helps on low contrast

    # Subpixel refine (keep it, but make accuracy less strict)
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 3
    params.cornerRefinementMaxIterations = 50
    params.cornerRefinementMinAccuracy = 0.05

    # Sometimes helps when board prints or lighting invert contrast locally
    params.detectInvertedMarker = True

    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
    if ids is None or len(ids) < 4:
        return False, None, None
    _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if ch_corners is None or ch_ids is None or len(ch_ids) < 6:
        return False, None, None
    return True, ch_corners, ch_ids


def calibrate_intrinsics(image_paths: List[str], mode: str, cols: int, rows: int, square: float, markerlength: float,
                         aruco_dict_id, save_undistorted: bool = False, out_dir: str = '.'):
    assert mode in ['checker', 'charuco']

    objpoints = []
    imgpoints = []
    used_paths = []

    if len(image_paths) == 0:
        raise RuntimeError('No images found to calibrate.')

    # Prepare boards
    if mode == 'checker':
        objp = create_board_object_points(cols, rows, square)
    else:  # ChArUco
        aruco_dict = aruco_dict_id  # this is already a Dictionary object
        print(f"Aruco Dict: {aruco_dict}")
        board = cv2.aruco.CharucoBoard_create(
            squaresX=cols, squaresY=rows,
            squareLength=square, markerLength=markerlength,
            dictionary=aruco_dict
        )

    img_size = None

    debug_dir = os.path.join(out_dir, 'debug_previews')
    os.makedirs(debug_dir, exist_ok=True)
    print(f"Saving debug images to: {debug_dir}")

    for p in sorted(image_paths):
        img = cv2.imread(p, cv2.IMREAD_UNCHANGED)
        if img is None:
            continue

        # 1) 8-bit grayscale for detection
        if img.ndim == 3:
            gray8 = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            if img.dtype == np.uint16:
                gray8 = (255.0 * (img.astype(np.float32) / 65535.0)).astype(np.uint8)
            else:
                gray8 = img.astype(np.uint8)

        if img_size is None:
            img_size = (gray8.shape[1], gray8.shape[0])

        # Create a color image for visualization output
        img_viz = cv2.cvtColor(gray8, cv2.COLOR_GRAY2BGR)

        # 2) Detection-only enhancement
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        gray_eq = clahe.apply(gray8)

        # For ArUco only: optional hard binarization (black/white)
        gray_bin = cv2.adaptiveThreshold(
            gray_eq, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,
            blockSize=21, C=7  # blockSize must be odd; tune 17..31, C ~ 5..9
        )

        cv2.imwrite('DEBUG_0_original_gray.png', gray8)
        cv2.imwrite('DEBUG_1_clahe_enhanced.png', gray_eq)
        cv2.imwrite('DEBUG_2_adaptive_binary.png', gray_bin)

        if mode == 'checker':
            # Use contrast-enhanced grayscale (NOT binary)
            ok, corners = find_checker_corners_2(gray_eq, cols, rows)
            if not ok:
                continue
            objpoints.append(objp.copy())
            imgpoints.append(corners)
            used_paths.append(p)

        else:
            # Try binary first (often best on low-res ToF), fall back to grayscale
            ok, ch_corners, ch_ids = detect_charuco(gray_bin, board, aruco_dict)
            if not ok:
                ok, ch_corners, ch_ids = detect_charuco(gray_eq, board, aruco_dict)
                if not ok:
                    continue
            objpoints.append((ch_corners, ch_ids))
            used_paths.append(p)


        if ok:
            cv2.putText(img_viz, "SUCCESS", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        else:
            cv2.putText(img_viz, "FAILED", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        
        # Save the debug image with a unique name
        base_filename = os.path.basename(p)
        output_path = os.path.join(debug_dir, f"debug_{base_filename}")
        cv2.imwrite(output_path, img_viz)


    if img_size is None or len(objpoints) < 8:
        raise RuntimeError(f'Insufficient valid views. Need at least ~8–12 good detections. current im_size: {img_size} and obj points: {len(objpoints)}')

    if mode == 'checker':
        # Standard calibrateCamera
        ret, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_size, None, None)
        reproj_rmse = float(ret)
    else:
        aruco_dict = aruco_dict_id
        board = cv2.aruco.CharucoBoard_create(cols, rows, square, markerlength, aruco_dict)
        
        # Unpack detections
        ch_corners_list = [c for c, _ in objpoints]
        ch_ids_list = [i for _, i in objpoints]
        ret, K, D, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
            charucoCorners=ch_corners_list,
            charucoIds=ch_ids_list,
            board=board,
            imageSize=img_size,
            cameraMatrix=None,
            distCoeffs=None
        )
        reproj_rmse = float(ret)

    result = {
        'image_width': img_size[0],
        'image_height': img_size[1],
        'K': K.tolist(),
        'D': D.squeeze().tolist(),
        'reproj_rmse': reproj_rmse,
        'num_views': len(used_paths)
    }

    if save_undistorted:
        out_prev = os.path.join(out_dir, 'undistorted_preview')
        os.makedirs(out_prev, exist_ok=True)

        K = np.asarray(K, dtype=np.float64)
        D = np.asarray(D, dtype=np.float64).ravel()

        for i, p in enumerate(used_paths[:8]):
            # Preserve bit depth / channels
            img = cv2.imread(p, cv2.IMREAD_UNCHANGED)
            if img is None:
                continue

            if img.ndim == 2:
                h, w = img.shape
            else:
                h, w = img.shape[:2]

            # Keep full FOV (no zoom)
            # Option A: use alpha=1
            newK, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), alpha=1)
            # Option B (even stricter no-rescale): newK = K.copy()

            map1, map2 = cv2.initUndistortRectifyMap(K, D, None, newK, (w, h), cv2.CV_32FC1)

            if img.ndim == 2:
                if img.dtype == np.uint16:
                    # remap in float to preserve precision
                    udf = cv2.remap(img.astype(np.float32), map1, map2, cv2.INTER_LINEAR)
                    ud16 = np.clip(udf, 0, 65535).astype(np.uint16)

                    # DO NOT crop by ROI
                    cv2.imwrite(os.path.join(out_prev, f'ud_{i:02d}.png'), ud16)

                    # 8-bit viewable preview (auto-stretched)
                    ud8 = cv2.normalize(ud16, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
                    cv2.imwrite(os.path.join(out_prev, f'ud_{i:02d}_8bit.png'), ud8)
                else:
                    ud = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
                    cv2.imwrite(os.path.join(out_prev, f'ud_{i:02d}.png'), ud)
            else:
                ud = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
                cv2.imwrite(os.path.join(out_prev, f'ud_{i:02d}.png'), ud)

    return result

def parse_aruco_dict(val: str):
    s = val.strip().upper().replace('5X5', '5X5').replace('4X4','4X4')  # noop but keeps intent clear
    s = s.replace('X', 'X')  # keep literal 'X'
    if hasattr(cv2.aruco, s):
        enum_id = getattr(cv2.aruco, s)
        return cv2.aruco.getPredefinedDictionary(enum_id)
    # also allow passing a raw int id
    try:
        enum_id = int(val)
        return cv2.aruco.getPredefinedDictionary(enum_id)
    except ValueError:
        pass
    raise ValueError(f"Unknown ArUco dictionary: {val}")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dataset', required=True, help='Path to session folder created by recorder')
    ap.add_argument('--mode', default='checker', choices=['checker', 'charuco'])

    # Checkerboard/ChArUco geometry
    ap.add_argument('--rgb_cols', type=int, default=6)
    ap.add_argument('--rgb_rows', type=int, default=9)
    ap.add_argument('--rgb_square', type=float, default=0.03)
    ap.add_argument('--markerlength', type=float, default=0.015)

    ap.add_argument('--tof_cols', type=int, default=6)
    ap.add_argument('--tof_rows', type=int, default=9)
    ap.add_argument('--tof_square', type=float, default=0.03)

    ap.add_argument('--aruco_dict', type=parse_aruco_dict,
                default="DICT_4X4_50",
                help="Predefined ArUco dictionary name, e.g. DICT_5X5_1000")
    ap.add_argument('--save_undistorted', type=int, default=1)

    args = ap.parse_args()

    rgb_paths = sorted(glob.glob(os.path.join(args.dataset, 'rgb', 'rgb_*.png')))
    tof_paths = sorted(glob.glob(os.path.join(args.dataset, 'tof_gray', 'gray_*.png')))

    if len(rgb_paths) == 0 or len(tof_paths) == 0:
        raise SystemExit(f"No images found. Did you run the recorder and point --dataset to the right session? # rgb: {len(rgb_paths)}, # depth: {len(tof_paths)}")

    os.makedirs(os.path.join(args.dataset, 'calib'), exist_ok=True)

    # --- RGB ---
    print("Starting calibration of RGB camera ...")
    rgb_result = calibrate_intrinsics(
        image_paths=rgb_paths,
        mode=args.mode,
        cols=args.rgb_cols,
        rows=args.rgb_rows,
        square=args.rgb_square,
        aruco_dict_id=args.aruco_dict,
        save_undistorted=bool(args.save_undistorted),
        markerlength=args.markerlength,
        out_dir=os.path.join(args.dataset, 'calib', 'rgb')
    )
    with open(os.path.join(args.dataset, 'calib', 'rgb_intrinsics.json'), 'w') as f:
        json.dump(rgb_result, f, indent=2)
    print(f"RGB calibration is finished")

    # --- ToF amplitude ---
    print("Starting calibration of ToF camera ...")
    tof_result = calibrate_intrinsics(
        image_paths=tof_paths,
        mode=args.mode,
        cols=args.tof_cols,
        rows=args.tof_rows,
        square=args.tof_square,
        aruco_dict_id=args.aruco_dict,
        save_undistorted=bool(args.save_undistorted),
        markerlength=args.markerlength,
        out_dir=os.path.join(args.dataset, 'calib', 'tof')
    )
    with open(os.path.join(args.dataset, 'calib', 'tof_intrinsics.json'), 'w') as f:
        json.dump(tof_result, f, indent=2)
    
    print(f"ToF calibration is finished")

    print('Saved:')
    print(os.path.join(args.dataset, 'calib', 'rgb_intrinsics.json'))
    print(os.path.join(args.dataset, 'calib', 'tof_intrinsics.json'))


if __name__ == '__main__':
    main()
