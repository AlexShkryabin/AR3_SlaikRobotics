# AR3_SlaikRobotics

Short reference for AR4 teleoperation and waypoint control.

## Build

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select ar4_teleop_gui --symlink-install
source install/setup.bash
```

## Launch

1. Start driver + MoveIt:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch annin_ar4_moveit_config moveit.launch.py ar_model:=mk4 moveit_servo:=True include_gripper:=False
```

2. Start GUI:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ar4_teleop_gui teleop_gui.launch.py
```

## GUI control quick guide

- Joint jog: sliders and +/- buttons for `joint_1..joint_6`
- Send target: sends current slider target to `joint_trajectory_controller`
- Cartesian jog: X/Y/Z and Mx/My/Mz through MoveIt Servo
- Waypoints:
	- Move arm to pose
	- Click `Add point (current pose)`
	- Repeat for next poses
	- Click `Start path (from first)`

## Smooth path behavior

Path playback sends one multi-point `JointTrajectory` for smooth interpolation.

Main motion tuning params in `teleop_gui.py`:
- `waypoint_segment_time`
- `waypoint_max_speed_deg_s`
- `waypoint_max_accel_deg_s2`

Lower speed/accel values make motion softer.

## Troubleshooting

- If MoveIt Servo package missing:

```bash
sudo apt install ros-jazzy-moveit-servo
```

- If motion is jerky on long moves:
	- increase `waypoint_segment_time`
	- reduce `waypoint_max_speed_deg_s`
	- reduce `waypoint_max_accel_deg_s2`