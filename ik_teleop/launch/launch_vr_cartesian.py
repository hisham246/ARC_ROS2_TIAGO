import os
from dataclasses import dataclass
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_pal.arg_utils import LaunchArgumentsBase, read_launch_argument
from moveit_configs_utils import MoveItConfigsBuilder
from tiago_pro_description.launch_arguments import TiagoProArgs

CONTROLLER_TOPIC = '/cartesian_impedance_controller_right/desired_pose'
GUARD_INPUT_TOPIC = '/vr_cartesian/desired_pose'


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    # the same robot arguments as launch_tiago_ik.py: they select the SRDF (collision rules) for the guard
    arm_type_right: DeclareLaunchArgument = TiagoProArgs.arm_type_right
    arm_type_left: DeclareLaunchArgument = TiagoProArgs.arm_type_left
    end_effector_right: DeclareLaunchArgument = TiagoProArgs.end_effector_right
    end_effector_left: DeclareLaunchArgument = TiagoProArgs.end_effector_left
    ft_sensor_right: DeclareLaunchArgument = TiagoProArgs.ft_sensor_right
    ft_sensor_left: DeclareLaunchArgument = TiagoProArgs.ft_sensor_left
    base_type: DeclareLaunchArgument = TiagoProArgs.base_type

    use_sim_time: DeclareLaunchArgument = DeclareLaunchArgument('use_sim_time', default_value='false')
    translation_only: DeclareLaunchArgument = DeclareLaunchArgument(
        'translation_only', default_value='false',
        description='Keep the end effector orientation, follow only the controller position')
    collision_guard: DeclareLaunchArgument = DeclareLaunchArgument(
        'collision_guard', default_value='true',
        description='Run the collision guard between the VR node and the impedance controller')


def start_nodes(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_guard = str(read_launch_argument('collision_guard', context)).lower() == 'true'

    config_bridge = os.path.join(
        get_package_share_directory('ik_teleop'), 'config', 'tiago_bridge.yaml')

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
        parameters=[{
            'use_sim_time': use_sim_time,
            'translation_only': ParameterValue(LaunchConfiguration('translation_only'), value_type=bool),
            'output_topic': GUARD_INPUT_TOPIC if use_guard else CONTROLLER_TOPIC,
        }],
    )

    gripper = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gripper_traj_passthrough'),
                'launch/tiago_gripper_passthrough.launch.xml')))

    actions = [bridge_node, teleop_node, gripper]

    if use_guard:
        srdf_input_args = {
            'arm_type_right': read_launch_argument('arm_type_right', context),
            'arm_type_left': read_launch_argument('arm_type_left', context),
            'end_effector_right': read_launch_argument('end_effector_right', context),
            'end_effector_left': read_launch_argument('end_effector_left', context),
            'ft_sensor_right': read_launch_argument('ft_sensor_right', context),
            'ft_sensor_left': read_launch_argument('ft_sensor_left', context),
            'base_type': read_launch_argument('base_type', context),
        }
        srdf_file_path = Path(os.path.join(
            get_package_share_directory('ik_teleop'), 'config', 'tiago_pro.srdf.xacro'))
        # robot_description is read from the /robot_description topic when the parameter is empty
        moveit_config = (
            MoveItConfigsBuilder('tiago_pro')
            .robot_description_semantic(file_path=srdf_file_path, mappings=srdf_input_args)
        )
        guard_node = Node(
            package='ik_teleop',
            name='cartesian_collision_guard',
            executable='cartesian_collision_guard',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time,
                 'input_topic': GUARD_INPUT_TOPIC,
                 'output_topic': CONTROLLER_TOPIC},
                moveit_config.to_dict(),
            ],
        )
        actions.append(guard_node)

    return actions


def generate_launch_description():
    ld = LaunchDescription()
    LaunchArguments().add_to_launch_description(ld)
    ld.add_action(OpaqueFunction(function=start_nodes))
    return ld
