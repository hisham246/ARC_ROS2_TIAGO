from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue

import launch_ros
import os
from glob import glob

def generate_launch_description():
	use_sim_time = LaunchConfiguration('use_sim_time', default='false')
	
	default_model_path = os.path.join(get_package_share_directory("nextage_fillie_open_description"),
		"urdf"
	)
	
	# default_urdf_path = os.path.join(get_package_share_directory("nextage_fillie_open_description"), "urdf", "nextage_fillie_open_gz.xacro")
	default_urdf_path = os.path.join(get_package_share_directory("nextage_fillie_open_description"), "urdf", "FillieOpen.urdf")

	
	gzclient = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
				os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
		),
		launch_arguments={"verbose": "true"}.items(), # "extra_gazebo_args": "-s libgazebo_ros_p3d.so"
	)
	gzserver = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
				os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
		),
		launch_arguments={"verbose": "true"}.items(),
	)

	robot_description_content = ParameterValue(
		Command(['xacro ', str(default_urdf_path)]),
		value_type=str
	)
	robot_state_publisher_node = launch_ros.actions.Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		output='both',
		parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description_content}]
	)
	spawn_entity = launch_ros.actions.Node(
		package="gazebo_ros",
		executable="spawn_entity.py",
		arguments=["-topic", "robot_description", "-entity", "FillieOpen"],
		output="screen",
	)
	effort_controller_config = os.path.join(
		get_package_share_directory("nextage_fillie_open_moveit_config"), "config", "ros2_controllers.yaml"
	)
	controller_manager = launch_ros.actions.Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[effort_controller_config],
            output='screen',
        )
	spawn_joint_state_broadcaster = launch_ros.actions.Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster', '-c', '/controller_manager'], # -c specifies the controller manager node name
            output='screen',
        )
	spawn_larm_controller = launch_ros.actions.Node(
		package="controller_manager",
		executable="spawner",
		arguments=["left_arm_controller", "-c", "/controller_manager"],
		output="screen",
	)
	spawn_rarm_controller = launch_ros.actions.Node(
		package="controller_manager",
		executable="spawner",
		arguments=["right_arm_controller", "-c", "/controller_manager"],
		output="screen",
	)
	spawn_torso_controller = launch_ros.actions.Node(
		package="controller_manager",
		executable="spawner",
		arguments=["torso_controller", "-c", "/controller_manager"],
		output="screen",
	)
	spawn_head_controller = launch_ros.actions.Node(
		package="controller_manager",
		executable="spawner",
		arguments=["head_controller", "-c", "/controller_manager"],
		output="screen",
	)
	gazebo_model_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[default_model_path])
	
	gaz_res_path = ""
	for path in glob("/usr/share/gazebo*"):
		gaz_res_path = gaz_res_path + path + ":"

	gazebo_resource_path = SetEnvironmentVariable(name='GAZEBO_RESOURCE_PATH', value=[gaz_res_path[:-1]])
	
	launch_description = launch.LaunchDescription([
		gazebo_model_path,
		gazebo_resource_path,
		launch.actions.DeclareLaunchArgument(
			'use_sim_time',
			default_value='false',
			description='Use simulation (Gazebo) clock if true'),
		controller_manager,
		spawn_joint_state_broadcaster,
		spawn_larm_controller,
		spawn_rarm_controller,
		spawn_head_controller,
		spawn_torso_controller,
		robot_state_publisher_node,
		gzclient,
		gzserver,
		spawn_entity,
		
	])

	return launch_description