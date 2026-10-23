import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_bridge = os.path.join(
        get_package_share_directory('ik_teleop'), 'config', 'tiago_bridge.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # /xr_input -> /grip/right (and the unused pose topics); the pose itself goes straight to the controller
    bridge_node = Node(
        package='ik_teleop',
        name='teleop_bridge',
        executable='teleop_msg_bridge',
        parameters=[config_bridge, {'relative_mode': False}],
    )

    teleop_node = Node(
        package='ik_teleop',
        name='vr_cartesian_teleop',
        executable='vr_cartesian_teleop.py',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    gripper = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gripper_traj_passthrough'),
                'launch/tiago_gripper_passthrough.launch.xml')))

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        bridge_node,
        teleop_node,
        gripper,
    ])
