# XMC_ROS2

- ROS2-Humble
- Infineon XMC1100
- Intel Realsense D415
- ToF Camera CamBoard pico flexx

## Run the node for steering
```
ros2 launch launch_node bringup_launch.py (device_port:=/dev/tty)*
```
`device_port` is defined according to the connection to Infineon XMC board

## Motion Data Streaming for ROS2

```bash
ros2 run psoc6_motion_bridge psoc6_motion_bridge --ros-args   -p port:=/dev/ttyACM0   -p baud:=230400   -p frame_id:=imu_link   -p topic:=/imu/data
```

### Get imu_link
```bash
ros2 run imu_filter_madgwick imu_filter_madgwick_node --ros-args -p use_mag:=false -p world_frame:=enu -r imu/data_raw:=/imu/data -r imu/data:=/imu/data_oriented
```
To monitor the value, echo the `/imu/data_oriented` topic.

## Run foxglove
```bash
ros2 run foxglove_bridge foxglove_bridge
```

**Open the visualizer**
```bash
foxglove-studio
```
and click `open connection`. The default is port:=8765.

**Connecting via ssh**
1. Connect to car via ssh (ip address in webex chat)
2. Launch foxbridge:
```bash
ros2 run foxglove_bridge foxglove_bridge_launch.xml port:=8765
```
3. In another terminal:
```bash
ssh -L 8765:localhost:8765 (device_name)@(ip_address)
```
4. Open foxglove app in local device: 1) choose open connection 2) choose ws://localhost:8765


## Build the robot model node (+ visualization)
```
sudo apt update
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
source /opt/ros/humble/setup.bash
source install/setup.bash
```
