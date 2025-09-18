# IK Based Teleop
This repo is for IK based Teleoperation. It is currently setup to work with the TiagoPro Docker Container. This will soon be ported to the Nextage Robot

# Installation
When pulling this repo ensure you are using the --recurse-submodules option.

```
git clone https://github.com/AdamSanderson93/ARC_ROS2_TIAGO.git --recurse-submodules
```

All required packages are installed using the install_deps.sh file.

```
sh install_deps.sh
```
Afterwards navigate to the src of the workspace and build the repo

```
colcon build --symlink-install
```

# Packages

## bio_ik
Optimization based IK solver. This is used by ik_teleop to plan the goal position of out end-effectors given multiple goals.

## cutom_teleop_messages
Contains custom message used to send in to the system. XRPoseMod is the only currently used message.

## gripper_traj_passthrough
Take a float32 message and turns it into a trajectory_msgs joint_trajectory. Used for grapper trajectories

## ik_teleop
The main package. This contains both the msg bridging nodes and the ik_planner. The ik_planner currently runs without sensor feedback (It assumes the robot with follow it properly. This will be updated shortly). The msg_string_bridge and string_msg_bridge take ROS messages and specially formatted strings and converts them. This allows them to be sent through webrtc data streams.

## opentera-webrtc-ros
This node is the main webrtc client. It connects to a signalling server to negotiate the webrtc connection. A custom signalling specification is used for calls and negotiating connections. This has been replecated on the unity side.

## webrtc_launch
This package contains nodes to strip out the peer information from recieved audio and video frames so they can be used in ROS directly. This allows videos to be processed or audio to be played via normal nodes. It also contains the main launch file for the webrtc interface.

# Running
Currently this requires the PAL Tiago Pro Docker container for robot geometry. This will be removed in the future.

Lauch Gazebo Node
```
ros2 launch tiago_pro_gazebo tiago_pro_gazebo.launch.py
```
In another terminal launch the movegroup, rviz, and ik_teleop nodes
```
ros2 launch ik_teleop launch_tiago_ik.py
```
Finally launch the webrtc interface
```
ros2 launch webrtc_launch tiago_pro.launch.xml
```

After this you should be able to launch the unity project. The webrtc and unity systems should connect from their.

Currently there is a gazebo bug that does not allow video to be transmitted. The video data is empty though the connection is made. Data does work so the robot will follow the operators motions.

# Install Problems

Audio Utils may have some issues with newer CPUs. Resolve this modify the file "make_openblas_unix.bash" for newer cpus...

'''
    make USE_THREAD=0 TARGET=SAPPHIRERAPIDS
'''

Appears to work