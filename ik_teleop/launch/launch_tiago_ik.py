# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_pal.arg_utils import read_launch_argument
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import AnyLaunchDescriptionSource

from moveit_configs_utils import MoveItConfigsBuilder
from launch_pal.arg_utils import LaunchArgumentsBase
from launch_pal.robot_arguments import CommonArgs
from tiago_pro_description.launch_arguments import TiagoProArgs
from tiago_pro_description.tiago_pro_launch_utils import get_tiago_pro_hw_suffix
from dataclasses import dataclass
from ament_index_python.packages import get_package_share_directory


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    arm_type_right: DeclareLaunchArgument = TiagoProArgs.arm_type_right
    arm_type_left: DeclareLaunchArgument = TiagoProArgs.arm_type_left
    end_effector_right: DeclareLaunchArgument = TiagoProArgs.end_effector_right
    end_effector_left: DeclareLaunchArgument = TiagoProArgs.end_effector_left
    ft_sensor_right: DeclareLaunchArgument = TiagoProArgs.ft_sensor_right
    ft_sensor_left: DeclareLaunchArgument = TiagoProArgs.ft_sensor_left
    base_type: DeclareLaunchArgument = TiagoProArgs.base_type

    use_sim_time: DeclareLaunchArgument = CommonArgs.use_sim_time
    use_sensor_manager_arg: DeclareLaunchArgument = CommonArgs.use_sensor_manager


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    launch_description.add_action(OpaqueFunction(function=start_move_group))
    return


def start_move_group(context, *args, **kwargs):

    arm_type_right = read_launch_argument('arm_type_right', context)
    arm_type_left = read_launch_argument('arm_type_left', context)
    end_effector_right = read_launch_argument('end_effector_right', context)
    end_effector_left = read_launch_argument('end_effector_left', context)
    use_sensor_manager = read_launch_argument('use_sensor_manager', context)

    hw_suffix = get_tiago_pro_hw_suffix(
        arm_right=arm_type_right,
        arm_left=arm_type_left,
        end_effector_right=end_effector_right,
        end_effector_left=end_effector_left)

    tiago_pro_moveit_path = get_package_share_directory("tiago_pro_moveit_config")

    srdf_file_path = Path(
        os.path.join(
            get_package_share_directory("ik_teleop"),
            "config",
            "tiago_pro.srdf.xacro",
        )
    )

    srdf_input_args = {
        'arm_type_right': read_launch_argument('arm_type_right', context),
        'arm_type_left': read_launch_argument('arm_type_left', context),
        'end_effector_right': read_launch_argument('end_effector_right', context),
        'end_effector_left': read_launch_argument('end_effector_left', context),
        'ft_sensor_right': read_launch_argument('ft_sensor_right', context),
        'ft_sensor_left': read_launch_argument('ft_sensor_left', context),
        "base_type": read_launch_argument("base_type", context),
    }

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = (
        os.path.join(
            tiago_pro_moveit_path,
            f'config/controllers/controllers{hw_suffix}.yaml'))

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # The robot description is read from the topic /robot_description if the parameter is empty
    moveit_config = (
        MoveItConfigsBuilder('tiago_pro')
        .robot_description_semantic(file_path=srdf_file_path, mappings=srdf_input_args)
        .robot_description_kinematics(file_path=os.path.join(
            get_package_share_directory("ik_teleop"), 'config', 'tiago_pro_kinematics.yaml'))
        .trajectory_execution(moveit_simple_controllers_path)
        .joint_limits(file_path=os.path.join(
            tiago_pro_moveit_path, 'config', 'joint_limits.yaml'))
        .planning_pipelines(pipelines=['ompl', 'chomp'], default_planning_pipeline='ompl')
        .planning_scene_monitor(planning_scene_monitor_parameters)
        .pilz_cartesian_limits(file_path=os.path.join(
            tiago_pro_moveit_path, 'config', 'pilz_cartesian_limits.yaml'))
    )

    if use_sensor_manager:
        # moveit_sensors path
        moveit_sensors_path = os.path.join(tiago_pro_moveit_path, 'config/sensors_3d.yaml')
        moveit_config.sensors_3d(moveit_sensors_path)

    rviz_moveit_config = moveit_config.to_moveit_configs()

    move_group_configuration = {
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'publish_robot_description_semantic': True,
        'robot_description_timeout': 120.0,
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
    ]

    # Start the actual move_group node/action server
    # run_move_group_node = Node(
    #     package='moveit_ros_move_group',
    #     executable='move_group',
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=move_group_params,
    # )

    config_control_node = os.path.join(
        get_package_share_directory('ik_teleop'),
        'config',
        'tiago_bio_ik_teleop.yaml'
        )

    control_node=Node(
        package = 'ik_teleop',
        name = 'IK_control_node',
        executable = 'control_node',
        output="screen",
        parameters = [{"use_sim_time": LaunchConfiguration('use_sim_time')},
                    config_control_node,
                    moveit_config.to_dict()]
    )

    config_bridge = os.path.join(
        get_package_share_directory('ik_teleop'),
        'config',
        'tiago_bridge.yaml'
        )
    
    bridge_node=Node(
        package = 'ik_teleop',
        name = 'teleop_bridge',
        executable = 'teleop_msg_bridge',
        parameters = [config_bridge]
    )

    # RViz
    rviz_base = os.path.join(get_package_share_directory(
        'ik_teleop'), 'config')
    rviz_full_config = os.path.join(rviz_base, 'tiago_pro.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', rviz_full_config],
        emulate_tty=True,
        parameters=[
            rviz_moveit_config.robot_description,
            rviz_moveit_config.robot_description_semantic,
            rviz_moveit_config.planning_pipelines,
            rviz_moveit_config.robot_description_kinematics,
        ],
    )

    # include another launch file
    launch_include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gripper_traj_passthrough'),
                'launch/tiago_gripper_passthrough.launch.xml'))
    )

    return [
        # run_move_group_node,
        control_node,
        bridge_node,
        rviz_node,
        launch_include]


def generate_launch_description():

    # Create the launch description and populate
    ld = LaunchDescription()
    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
