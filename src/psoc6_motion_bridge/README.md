# Motion Data Streaming for ROS2

```bash
ros2 run psoc6_motion_bridge psoc6_motion_bridge --ros-args   -p port:=/dev/ttyACM0   -p baud:=230400   -p frame_id:=imu_link   -p topic:=/imu/data
```

Get imu_link
```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false -p world_frame:=enu -r imu/data_raw:=/imu/data -r imu/data:=/imu/data_oriented
```