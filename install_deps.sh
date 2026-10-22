#!/bin/bash
# Dependencies for the VR teleop stack: ik_teleop, bio_ik, fp, custom_teleop_messages,
# gripper_traj_passthrough and ROS-TCP-Endpoint. Assumes ROS 2 Humble and the PAL Tiago Pro container.
set -e

# build tools and libraries (bio_ik, fp)
sudo apt install -y ros-dev-tools librange-v3-dev libfmt-dev libeigen3-dev libfcl-dev

# ROS packages
sudo apt install -y ros-humble-moveit ros-humble-backward-ros ros-humble-tf2-kdl ros-humble-kdl-parser

# colcon --symlink-install needs a setuptools that still supports `setup.py develop --editable` (< 80)
python3 -m pip install --user "setuptools==58.2.0"

# clone this repo with --recurse-submodules (ROS-TCP-Endpoint is one of them)
