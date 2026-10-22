# IK Based Teleop
This repo is for IK based VR teleoperation of the TIAGo Pro. A Unity app on a VR headset (see [Unity_Bridge_VR](https://github.com/AdamSanderson93/Unity_Bridge_VR)) sends controller poses over ROS-TCP to an IK planner, which drives the robot's arms, torso and head. It is currently set up to work with the PAL TIAGo Pro Docker container.

# Installation
When pulling this repo ensure you are using the --recurse-submodules option.

```
git clone https://github.com/AdamSanderson93/ARC_ROS2_TIAGO.git --recurse-submodules
```

Install the dependencies (ROS 2 Humble is assumed):

```
bash install_deps.sh
```

Then build from the workspace root and source it:

```
colcon build --symlink-install
source install/setup.bash
```

# Packages

## bio_ik
Optimization based IK solver. This is used by ik_teleop to plan the goal position of the end-effectors given multiple goals.

## fp
Functional programming helper library that bio_ik depends on.

## custom_teleop_messages
Custom messages sent by the XR system. `XRMod` (left, right and head poses plus the two squeeze values) is the message published on `/xr_input`. `BoolTrigger` is the service type the Unity app uses for `toggle_tracking` and `stop_robot`.

## gripper_traj_passthrough
Takes a float32 message and turns it into a trajectory_msgs joint_trajectory. Used for gripper trajectories.

## ik_teleop
The main package. It contains `teleop_msg_bridge`, which turns `/xr_input` into per-device pose commands, and `control_node`, which runs the IK planner. The ik_planner currently runs without sensor feedback (it assumes the robot will follow it properly, except for the optional `allowable_lag` per group). Configuration is in `ik_teleop/config`: `tiago_bridge.yaml` (topics and pose offsets) and `tiago_bio_ik_teleop.yaml` (planner groups and weights).

## ROS-TCP-Endpoint
Unity Robotics Hub endpoint (`main-ros2` branch). The Unity app connects to it over TCP on port 10000.

# Running
Currently this requires the PAL TIAGo Pro Docker container for robot geometry. This will be removed in the future. Source the workspace in each terminal.

Launch the Gazebo node
```
ros2 launch tiago_pro_gazebo tiago_pro_gazebo.launch.py
```
Wait for the arms to finish tucking, then launch the movegroup, rviz and ik_teleop nodes in another terminal
```
ros2 launch ik_teleop launch_tiago_ik.py
```
Finally start the TCP endpoint for Unity
```
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=0.0.0.0
```

In the Unity project, set the ROS IP address (Robotics > ROS Settings) to the machine running the endpoint, port 10000, protocol ROS2. Build it to the headset and launch the app. The endpoint prints `Connection from <headset ip>` and registers `/xr_input` and `/cmd_vel`.

To check the data path:
```
ros2 topic hz /xr_input
ros2 topic hz /left_arm_cmd
ros2 topic echo /arm_left_controller/joint_trajectory --once
```

# Known issues
- The bridge drops a message if any of the three poses is at the origin, so both controllers must be tracked for the arms to move.
- The planner rejects any solution in which the grippers collide (`Planned End State Colliding`), for example when both controllers are held close together.
- Nothing in this repo serves the `toggle_tracking` and `stop_robot` services that the Unity app calls, so those buttons currently have no effect.
- Gazebo's camera is published best-effort while the TCP endpoint subscribes reliably, so no video reaches Unity in simulation.

# Install Problems

`colcon build --symlink-install` fails on the Python packages with `option --editable not recognized` if setuptools is version 80 or newer. Pin it:

```
python3 -m pip install "setuptools==58.2.0"
```
