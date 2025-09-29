#!/usr/bin/env python3
"""
Extrinsic calibration between Pico Flexx (ToF amplitude/depth) and RealSense D415 RGB
using a calibration board. Supports checkerboard or ChArUco.

Workflow (Route A = board poses per camera):
 1) For each synchronized frame, detect the board in RGB and ToF amplitude.
 2) Using each camera's *intrinsics* (from previous step), solve PnP to get
    T_rgb<-board and T_tof<-board per frame.
 3) Compose per-frame extrinsics: T_rgb<-tof = T_rgb<-board * inv(T_tof<-board).
 4) Robustly average over all frames to get the final T_rgb<-tof.
 5) Optionally, colorize one ToF depth frame using RGB and the estimated extrinsics
    to write a quick sanity-check PLY.

Why not stereoCalibrate? The RGB and ToF images may have different sizes.
This pose-composition route avoids that constraint while using the same math.

Usage example:
  python3 calib_extrinsics.py --dataset /path/to/session \
    --mode checker --cols 6 --rows 9 --square 0.03 \
    --aruco_dict 4 --save_colored_ply 1 --ply_index 0

Inputs expected (from the earlier recorder):
  <dataset>/rgb/rgb_*.png
  <dataset>/tof_amp/amp_*.png
  <dataset>/tof_depth/depth_*.npy (optional but needed to write PLY)
  <dataset>/calib/rgb_intrinsics.json
  <dataset>/calib/tof_intrinsics.json
  <dataset>/manifest.csv

Outputs:
  <dataset>/calib/extrinsics_rgb_from_tof.json   # contains R(3x3), t(3,), and stats
  <dataset>/calib/tof_rgb_preview.ply            # optional colorized point cloud
"""

import os
import csv
import json
import glob
import argparse
from typing import Tuple, List, Dict

import numpy as np
import cv2


def load_intrinsics(json_path: str) -> Tuple[np.ndarray, np.ndarray]:
    with open(json_path, 'r') as f:
        data = json.load(f)
    K = np.array(data['K'], dtype=np.float64).reshape(3,3)
    D = np.array(data['D'], dtype=np.float64).ravel()
    return K, D


def checkerboard_object_points(cols: int, rows: int, square: float) -> np.ndarray:
    objp = np.zeros((rows * cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= float(square)
    return objp.astype(np.float32)


def find_checker(gray: np.ndarray, cols: int, rows: int) -> Tuple[bool, np.ndarray]:
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not ok:
        return False, None
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-3)
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
    return True, corners


def detect_charuco(gray: np.ndarray, board: cv2.aruco_CharucoBoard, aruco_dict) -> Tuple[bool, np.ndarray, np.ndarray]:
    params = cv2.aruco.DetectorParameters_create()
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)
    if ids is None or len(ids) < 4:
        return False, None, None
    _, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(corners, ids, gray, board)
    if ch_corners is None or ch_ids is None or len(ch_ids) < 6:
        return False, None, None
    return True, ch_corners, ch_ids.ravel()


def pnp_pose(objp: np.ndarray, imgp: np.ndarray, K: np.ndarray, D: np.ndarray) -> Tuple[np.ndarray, np.ndarray, bool]:
    # Use SOLVEPNP_IPPE_SQUARE for near-planar boards if square, otherwise RANSAC PnP
    # Here: RANSAC for robustness
    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        objectPoints=objp,
        imagePoints=imgp,
        cameraMatrix=K,
        distCoeffs=D,
        flags=cv2.SOLVEPNP_ITERATIVE,
        reprojectionError=3.0,
        iterationsCount=200,
        confidence=0.99
    )
    return rvec, tvec, ok


def Rt_from_rvec_tvec(rvec: np.ndarray, tvec: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)
    return R, t


def invert_RT(R: np.ndarray, t: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    Rinv = R.T
    tinv = -Rinv @ t
    return Rinv, tinv


def compose_RT(Ra, ta, Rb, tb) -> Tuple[np.ndarray, np.ndarray]:
    R = Ra @ Rb
    t = Ra @ tb + ta
    return R, t


def rotation_mean_from_rvecs(rvecs: List[np.ndarray]) -> np.ndarray:
    # Simple Lie-algebra mean: average rotation vectors, then Rodrigues
    vecs = np.stack([rv.reshape(3) for rv in rvecs], axis=0)
    mean_vec = np.mean(vecs, axis=0)
    R, _ = cv2.Rodrigues(mean_vec)
    return R


def write_json(path: str, data: Dict):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        json.dump(data, f, indent=2)


def bilinear_sample(img: np.ndarray, uv: np.ndarray) -> np.ndarray:
    h, w = img.shape[:2]
    x = uv[:, 0]
    y = uv[:, 1]
    x0 = np.clip(np.floor(x).astype(int), 0, w - 1)
    y0 = np.clip(np.floor(y).astype(int), 0, h - 1)
    x1 = np.clip(x0 + 1, 0, w - 1)
    y1 = np.clip(y0 + 1, 0, h - 1)
    wa = (x1 - x) * (y1 - y)
    wb = (x - x0) * (y1 - y)
    wc = (x1 - x) * (y - y0)
    wd = (x - x0) * (y - y0)
    Ia = img[y0, x0]
    Ib = img[y0, x1]
    Ic = img[y1, x0]
    Id = img[y1, x1]
    out = (Ia * wa[:, None] + Ib * wb[:, None] + Ic * wc[:, None] + Id * wd[:, None]).astype(np.uint8)
    return out


def save_ply(path: str, pts: np.ndarray, colors: np.ndarray):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, 'w') as f:
        f.write('ply\nformat ascii 1.0\n')
        f.write(f'element vertex {pts.shape[0]}\n')
        f.write('property float x\nproperty float y\nproperty float z\n')
        f.write('property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n')
        for (x, y, z), (r, g, b) in zip(pts, colors):
            f.write(f'{x} {y} {z} {int(r)} {int(g)} {int(b)}\n')


def colorize_one_frame(dataset: str, K_rgb, D_rgb, K_tof, D_tof, R_rt, t_rt, frame_index: int):
    rgb_path = os.path.join(dataset, 'rgb', f'rgb_{frame_index:06d}.png')
    depth_npy = os.path.join(dataset, 'tof_depth', f'depth_{frame_index:06d}.npy')
    if not (os.path.isfile(rgb_path) and os.path.isfile(depth_npy)):
        print('[WARN] Skipping PLY preview (missing rgb/depth files).')
        return

    rgb = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
    depth = np.load(depth_npy)  # expect meters float32; if not, adapt

    h, w = depth.shape
    us, vs = np.meshgrid(np.arange(w), np.arange(h))
    pix = np.stack([us.reshape(-1), vs.reshape(-1)], axis=1).astype(np.float32)

    # Back-project ToF
    ones = np.ones((pix.shape[0], 1), dtype=np.float32)
    pix_h = np.concatenate([pix, ones], axis=1)
    Kinv_tof = np.linalg.inv(K_tof)
    rays = (Kinv_tof @ pix_h.T).T  # Nx3
    z = depth.reshape(-1, 1).astype(np.float32)
    pts_tof = rays * z  # Nx3 in ToF frame

    # Transform to RGB frame
    pts_rgb = (R_rt @ pts_tof.T).T + t_rt.reshape(1, 3)

    # Project to RGB with distortion via cv2.projectPoints (batch in chunks)
    rvec_zero = np.zeros((3, 1), dtype=np.float64)
    tvec_zero = np.zeros((3, 1), dtype=np.float64)
    colors = np.zeros_like(pts_tof, dtype=np.uint8)

    # Create validity mask (Z>0)
    valid = pts_rgb[:, 2] > 0
    idxs = np.where(valid)[0]

    # Chunking to avoid huge allocations
    chunk = 200000
    uv = np.zeros((idxs.size, 2), dtype=np.float32)
    for s in range(0, idxs.size, chunk):
        sel = idxs[s:s+chunk]
        obj = pts_rgb[sel].astype(np.float64).reshape(-1, 1, 3)
        uv_proj, _ = cv2.projectPoints(obj, rvec_zero, tvec_zero, K_rgb, D_rgb)
        uv[s:s+sel.size] = uv_proj.reshape(-1, 2).astype(np.float32)

    # Sample colors for valid points only
    rgb_colors = bilinear_sample(rgb, uv)
    colors[idxs] = rgb_colors

    # Keep only valid with color inside image
    H, W = rgb.shape[:2]
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < W-1) & (uv[:, 1] >= 0) & (uv[:, 1] < H-1)
    keep = valid.copy()
    keep[idxs] &= inside

    pts_keep = pts_tof[keep]
    cols_keep = colors[keep]

    out_ply = os.path.join(dataset, 'calib', 'tof_rgb_preview.ply')
    save_ply(out_ply, pts_keep, cols_keep)
    print(f'[OK] Wrote colorized preview PLY -> {out_ply} (points: {pts_keep.shape[0]})')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--dataset', required=True, help='Path to session folder')
    ap.add_argument('--mode', default='checker', choices=['checker', 'charuco'])
    ap.add_argument('--cols', type=int, default=6, help='checker: inner corners across; charuco: squaresX')
    ap.add_argument('--rows', type=int, default=9, help='checker: inner corners down; charuco: squaresY')
    ap.add_argument('--square', type=float, default=0.03, help='square size in meters')
    ap.add_argument('--aruco_dict', type=int, default=4, help='cv2.aruco.DICT_* id, e.g., 4=DICT_4X4_50')
    ap.add_argument('--min_pairs', type=int, default=8, help='min valid pairs required')
    ap.add_argument('--save_colored_ply', type=int, default=1)
    ap.add_argument('--ply_index', type=int, default=0, help='frame index to colorize')

    args = ap.parse_args()

    # Intrinsics
    K_rgb, D_rgb = load_intrinsics(os.path.join(args.dataset, 'calib', 'rgb_intrinsics.json'))
    K_tof, D_tof = load_intrinsics(os.path.join(args.dataset, 'calib', 'tof_intrinsics.json'))

    # Prepare board
    if args.mode == 'checker':
        objp_checker = checkerboard_object_points(args.cols, args.rows, args.square)
    else:
        aruco_dict = cv2.aruco.getPredefinedDictionary(args.aruco_dict)
        board = cv2.aruco.CharucoBoard_create(args.cols, args.rows, args.square, 0.7*args.square, aruco_dict)

    # Read synchronization files
    # index,stamp_ns,rgb_path,amp_path,depth_path,rgb_frame_id,tof_frame_id
    sync_path = os.path.join(args.dataset, 'sync.csv')
    
    pairs = []
    with open(sync_path, 'r') as f:
        rdr = csv.DictReader(f)
        for row in rdr:
            rgb_path = os.path.join(args.dataset, row['rgb_path'])
            amp_path = os.path.join(args.dataset, row['depth_path']).replace('tof_depth/depth_', 'tof_gray/tof_gray')  # recorder wrote depth_path; we derive amp path
            # Prefer explicit amp file if exists
            amp_guess = os.path.join(args.dataset, 'tof_gray', os.path.basename(amp_path).replace('depth_', 'gray_'))
            if os.path.isfile(amp_guess):
                amp_path = amp_guess
            idx = int(row['index'])
            pairs.append((idx, rgb_path, amp_path))

    rvecs_rel = []
    tvecs_rel = []
    used_ids = []

    for idx, rgb_path, amp_path in pairs:
        img_rgb = cv2.imread(rgb_path, cv2.IMREAD_COLOR)
        img_amp = cv2.imread(amp_path, cv2.IMREAD_UNCHANGED)
        if img_rgb is None or img_amp is None:
            continue
        
        # convert to gray scale
        gray_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_BGR2GRAY)
        if img_amp.ndim == 3:
            gray_amp = cv2.cvtColor(img_amp, cv2.COLOR_BGR2GRAY)
        else:
            # normalize 16U for detection if needed
            gray_amp = img_amp
            if gray_amp.dtype == np.uint16:
                gray_amp = (255.0 * (gray_amp.astype(np.float32) / 65535.0)).astype(np.uint8)

        if args.mode == 'checker':
            ok1, corners_rgb = find_checker(gray_rgb, args.cols, args.rows)
            ok2, corners_amp = find_checker(gray_amp, args.cols, args.rows)
            if not (ok1 and ok2):
                continue
            rvec_rgb, tvec_rgb, okr = pnp_pose(objp_checker, corners_rgb, K_rgb, D_rgb)
            rvec_tof, tvec_tof, okt = pnp_pose(objp_checker, corners_amp, K_tof, D_tof)
            if not (okr and okt):
                continue
            R_rgb, t_rgb = Rt_from_rvec_tvec(rvec_rgb, tvec_rgb)
            R_tof, t_tof = Rt_from_rvec_tvec(rvec_tof, tvec_tof)

            # T_rgb<-tof = T_rgb<-board * inv(T_tof<-board)
            R_tb_inv, t_tb_inv = invert_RT(R_tof, t_tof)
            R_rel, t_rel = compose_RT(R_rgb, t_rgb, R_tb_inv, t_tb_inv)

        else:  # charuco
            aruco_dict = cv2.aruco.getPredefinedDictionary(args.aruco_dict)
            board = cv2.aruco.CharucoBoard_create(args.cols, args.rows, args.square, 0.7*args.square, aruco_dict)
            ok1, ch_corners_rgb, ch_ids_rgb = detect_charuco(gray_rgb, board, aruco_dict)
            ok2, ch_corners_tof, ch_ids_tof = detect_charuco(gray_amp, board, aruco_dict)
            if not (ok1 and ok2):
                continue
            # match IDs intersection
            ids_rgb = ch_ids_rgb.astype(int)
            ids_tof = ch_ids_tof.astype(int)
            common = np.intersect1d(ids_rgb, ids_tof)
            if common.size < 6:
                continue
            # build correspondences in the same order
            def select_by_ids(corners, ids, wanted):
                out = []
                for wid in wanted:
                    idxs = np.where(ids == wid)[0]
                    if idxs.size == 0:
                        continue
                    out.append(corners[idxs[0]])
                return np.array(out).reshape(-1,1,2)
            corners_rgb_sel = select_by_ids(ch_corners_rgb, ids_rgb, common)
            corners_tof_sel = select_by_ids(ch_corners_tof, ids_tof, common)
            # 3D object points for those charuco IDs
            objp = board.chessboardCorners[common].astype(np.float32)

            rvec_rgb, tvec_rgb, okr = pnp_pose(objp, corners_rgb_sel, K_rgb, D_rgb)
            rvec_tof, tvec_tof, okt = pnp_pose(objp, corners_tof_sel, K_tof, D_tof)
            if not (okr and okt):
                continue
            R_rgb, t_rgb = Rt_from_rvec_tvec(rvec_rgb, tvec_rgb)
            R_tof, t_tof = Rt_from_rvec_tvec(rvec_tof, tvec_tof)
            R_tb_inv, t_tb_inv = invert_RT(R_tof, t_tof)
            R_rel, t_rel = compose_RT(R_rgb, t_rgb, R_tb_inv, t_tb_inv)

        # store relative as rvec,tvec for averaging
        rvec_rel, _ = cv2.Rodrigues(R_rel)
        rvecs_rel.append(rvec_rel.reshape(3))
        tvecs_rel.append(t_rel.reshape(3))
        used_ids.append(idx)

    if len(used_ids) < args.min_pairs:
        raise SystemExit(f'Insufficient valid pairs: {len(used_ids)} < {args.min_pairs}')

    rvecs_rel = np.array(rvecs_rel)
    tvecs_rel = np.array(tvecs_rel)
    # robust center: median, then refine by mean
    rvec_med = np.median(rvecs_rel, axis=0)
    R_med, _ = cv2.Rodrigues(rvec_med.astype(np.float64))
    t_med = np.median(tvecs_rel, axis=0)

    # Optional refine by simple mean near the median
    R_mean = rotation_mean_from_rvecs([r.reshape(3,1) for r in rvecs_rel])
    t_mean = np.mean(tvecs_rel, axis=0)

    # Choose mean as final, but report both
    R_final = R_mean
    t_final = t_mean

    # Residuals: angular distance and translation spread
    def rot_angle(Ra, Rb):
        R = Ra.T @ Rb
        angle = np.arccos(np.clip((np.trace(R) - 1)/2, -1, 1))
        return float(angle)

    ang_res = [rot_angle(R_final, cv2.Rodrigues(r.reshape(3,1))[0]) for r in rvecs_rel]
    trans_res = [float(np.linalg.norm(t_final - t)) for t in tvecs_rel]

    out = {
        'R': R_final.tolist(),
        't': t_final.tolist(),
        'frames_used': int(len(used_ids)),
        'used_indices': [int(i) for i in used_ids],
        'rotation_median_deg': float(np.rad2deg(np.median(ang_res))),
        'rotation_mean_deg': float(np.rad2deg(np.mean(ang_res))),
        'translation_median_m': float(np.median(trans_res)),
        'translation_mean_m': float(np.mean(trans_res)),
        'R_median': R_med.tolist(),
        't_median': t_med.tolist()
    }

    out_path = os.path.join(args.dataset, 'calib', 'extrinsics_rgb_from_tof.json')
    write_json(out_path, out)
    print(f'[OK] Saved extrinsics -> {out_path}')

    if args.save_colored_ply:
        colorize_one_frame(args.dataset, K_rgb, D_rgb, K_tof, D_tof, R_final, t_final, args.ply_index)


if __name__ == '__main__':
    main()
