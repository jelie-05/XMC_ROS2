# Camera Calibration

## Camera Info
Lens Parameters from device
```ini
cx = 103.8418
cy = 87.3468
fx = 203.3543
fy = 203.3543
k1 = 1.1038
k2 = -8.3290
k3 = 13.8729
p1 = 3.7407e-15
p2 = 4.1990e-15
```

## Recording
```
ros2 run calib_recorder calib_recorder \
  --ros-args -p rgb_topic:=/camera/camera/color/image_raw \
  -p rgb_info_topic:=/camera/camera/color/camera_info \
  -p tof_gray_topic:=/royale/gray_image \
  -p out_dir:=$HOME/XMC_ROS2/calib_data \
  -p session:=realsense_pico_charuco_$(date +%Y%m%d) \
  -p save_rate_hz:=2.0 -p sync_slop:=0.03 -p queue_size:=15 -p max_frames:=90
```
For the visualization, run rviz2 to ensure the whole pattern is captured.

## Intrinsic Calibration 
```
python3 tools/calib_intrinsics.py \
  --dataset calib_data/realsense_pico_charuco_20250919_3 \
  --mode charuco \
  --rgb_cols 6 --rgb_rows 4 --rgb_square 0.044 \
  --tof_cols 6 --tof_rows 4 --tof_square 0.044 \
  --markerlength 0.031 \
  --aruco_dict DICT_5X5_1000 \
  --save_undistorted 1
```
`*_cols` and `*_rows` correspond to number of square in horizontal and vertical direction. `*_square` corresponds to checkerboard square size and `markerlength` to ArUco marker size. `aruco_dict` is the dictionary of the ArUco pattern.

### Note: current problem: the gray scale corners are not detected well.
The parameters in `detect_charuco` could be adjusted to ease the tolerance, but didnt work yet.