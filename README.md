# Rover ROS2 Package 

ROS 2 package full of resources for robot development.

## Scripts
- `display_heading.py` - overlays the EKF heading on depth camera frames and republishes the annotated image.
- `gps_frame_fixer.py` - rewrites `/gps/fix` messages with `gps_link` frame and updated timestamps on `/gps/fix_corrected`.
- `panorama_stitcher.py` - sweeps the pan joint through preset angles, captures frames, and stitches them into `panorama.jpg`.
- `panorama_stitcher_with_heading.py` - performs the same sweep but adds compass heading overlay and saves numbered panoramas.
- `path_publisher.py` - subscribes to `/odometry/global`, accumulates poses, and publishes a `Path` on `/robot_path`.
- `yolo_depth_v1.py` - runs YOLOv11 on RGB + depth, estimates 3D boxes/sizes, and publishes `Detection3DArray` plus debug images.
- `yolo_depth_v2.py` - higher-accuracy YOLO + depth version with median-filtered depth sampling and tighter size estimation.
- `yolo_pc.py` - YOLO + depth point-cloud workflow that classifies shape (round/box) and derives 3D dimensions from dense depth points.
- `yolo11n.pt` - bundled YOLOv11n model weights used by the YOLO scripts.

## Launch files and assets
- `launch/display.launch.py` starts RViz, `robot_state_publisher`, and Gazebo with `world/my_world.sdf`, loading the servo camera URDF and EKF localization.
- `launch/map_display.launch.py` brings up Gazebo plus the full localization stack (dual EKFs + navsat), GPS restamper, heading overlay, path publisher, YOLO depth node, teleop, and Foxglove for visualization.
- `launch/localization.launch.py` runs just the localization pipeline (two EKFs and navsat transform) against live topics.
- `launch/moon.launch.py` spawns the robot into `world/my_moon_world.sdf` with EKF+navsat, GPS restamping, and heading display; sets `GAZEBO_MODEL_PATH` to the bundled models.
- `launch/servo.launch.py` loads the servo-camera URDF into `world/my_soccer_world.sdf`, then spawns ros2_control controllers for joint states and the camera pan position command.

- Gazebo worlds live in `world/` (`my_world.sdf`, `my_moon_world.sdf`, `my_soccer_world.sdf`, `moon_world.sdf`, `soccer.sdf`).

- Localization and nav configs are in `config/` (EKF local/global variants, navsat transform, Nav2 params), with ros2_control defined in `config/sam_bot_ros2_control.yaml`.

## URDFs and SDFs
- `src/description/sam_bot_description.urdf` uses xacro properties/macros to define the base, wheels, caster, and inertias. Gazebo plugins provide sensors and mobility: `libgazebo_ros_imu_sensor.so` publishes `/imu`, `libgazebo_ros_gps_sensor.so` publishes GPS, `libgazebo_ros_diff_drive.so` drives the left/right wheels, `libgazebo_ros_ray_sensor.so` outputs `/scan`, and `libgazebo_ros_camera.so` hosts a depth camera frame.

- `src/description/sam_bot_servo_camera.urdf` extends the base model with a pan joint for the camera. It reuses the same xacro helpers, adds Gazebo IMU/GPS/LiDAR plugins, and wires ros2_control via `<ros2_control>` using the `gazebo_ros2_control/GazeboSystem` hardware plugin. The accompanying `gazebo_ros2_control` Gazebo plugin loads controllers from `config/sam_bot_ros2_control.yaml`, exposing `camera_pan_joint` with position command plus position/velocity state interfaces.

- Gazebo worlds live in `world/` (`my_world.sdf`, `my_moon_world.sdf`, `my_soccer_world.sdf`, `moon_world.sdf`, `soccer.sdf`).


## Requirements
- ROS 2 Humble
- Ubuntu 22.04
- Gazebo (classic) with `gazebo_ros` integration
- OpenCV
- `teleop_twist_keyboard` for keyboard control

- Foxglove Bridge (optional, for `map_display.launch.py`)
- Ultralytics YOLO (optional, for `yolo_depth_v1.py`)

## Build
1) Install base tools (rosdep handles ROS package deps):
```bash
sudo apt update
sudo apt install python3-rosdep python3-opencv
pip install ultralytics # Optional, may take some time
sudo rosdep init   # safe to rerun; ignore "already initialized" warnings
rosdep update
```

2) Create a workspace and clone this package:
```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/nav2_ws/src
cd ~/nav2_ws/src
git clone https://github.com/tonitonitonitoni/asimov2026.git
```

3) Install package dependencies via rosdep:
```bash
cd ~/nav2_ws
rosdep install --from-paths src --ignore-src -r -y
```

4) Build and source the overlay:
```bash
colcon build --symlink-install
source install/setup.bash
```

## Play
5) Launch the servo camera world.
`ros2 launch sam_bot_description servo.launch.py` 
6) Take a panorama photo.
`ros2 run sam_bot_description panorama_stitcher_with_heading` 
7) Estimate depth from a detected object 
`ros2 run sam_bot_description yolo_depth_v1`
8) Bring up the entire localization stack and prepare for display in Foxglove 
`ros2 launch sam_bot_description map_display.launch.py` 
