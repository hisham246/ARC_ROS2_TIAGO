import os

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from tiago_dual_description.tiago_dual_launch_utils import get_tiago_dual_hw_suffix
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    ld = LaunchDescription()

    arm_type_right = 'tiago-arm'
    arm_type_left= 'tiago-arm'
    end_effector_right = 'pal-gripper'
    end_effector_left = 'pal-gripper'
    ft_sensor_right = 'schunk-ft'
    ft_sensor_left = 'schunk-ft'
    base_type = 'pmb2'

    hw_suffix = get_tiago_dual_hw_suffix(
        arm_right=arm_type_right,
        arm_left=arm_type_left,
        end_effector_right=end_effector_right,
        end_effector_left=end_effector_left)

    srdf_file_path = Path(
        os.path.join(
            get_package_share_directory("tiago_dual_moveit_config"),
            "config", "srdf",
            "tiago_dual.srdf.xacro",
        )
    )

    srdf_input_args = {
        "arm_type_right": arm_type_right,
        "arm_type_left": arm_type_left,
        "end_effector_right": end_effector_right,
        "end_effector_left": end_effector_left,
        "ft_sensor_right": ft_sensor_right,
        "ft_sensor_left": ft_sensor_left,
        "base_type": base_type,
    }

    # Trajectory Execution Functionality
    moveit_simple_controllers_path = (get_package_share_directory("tiago_dual_moveit_config")+
        f'/config/controllers/controllers{hw_suffix}.yaml')

    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
    }

    # The robot description is read from the topic /robot_description if the parameter is empty
    moveit_config = (
        MoveItConfigsBuilder('tiago_dual')
        .robot_description_semantic(file_path=srdf_file_path, mappings=srdf_input_args)
        .robot_description_kinematics(file_path=os.path.join(get_package_share_directory("tiago_dual_moveit_config"),
            'config', 'kinematics_kdl.yaml'))
        .trajectory_execution(moveit_simple_controllers_path)
        .planning_pipelines(pipelines=['ompl'])
        .planning_scene_monitor(planning_scene_monitor_parameters)
        .pilz_cartesian_limits(file_path=os.path.join(get_package_share_directory("tiago_dual_moveit_config"),
            'config', 'pilz_cartesian_limits.yaml'))
    )

    if True:
        # moveit_sensors path
        moveit_sensors_path = 'config/sensors_3d.yaml'
        moveit_config.sensors_3d(moveit_sensors_path)

    moveit_config.to_moveit_configs()

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
        parameters = [{"use_sim_time": True},
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

    ld.add_action(control_node)
    ld.add_action(bridge_node)


    return ld