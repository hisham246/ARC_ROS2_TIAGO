from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

def generate_launch_description():
	use_sim_time = LaunchConfiguration('use_sim_time', default='false')
	
	default_model_path = os.path.join(get_package_share_directory("nextage_fillie_open_description"),
		"urdf"
	)
	
	default_urdf_path = os.path.join(get_package_share_directory("nextage_fillie_open_description"), "urdf", "nextage_fillie_open_gz.xacro")
		
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
	robot_state_publisher_node = launch_ros.actions.Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		output='both',
		parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
	)
	spawn_entity = launch_ros.actions.Node(
		package="gazebo_ros",
		executable="spawn_entity.py",
		arguments=["-topic", "robot_description", "-entity", "FillieOpen"],
		output="screen",
	)
	effort_controller_config = os.path.join(
		get_package_share_directory("nextage_fillie_open_gazebo"), "config", "ros_control.yaml"
	)
	spawn_controller = launch_ros.actions.Node(
		package="controller_manager",
		executable="spawner.py",
		arguments=["joint_group_position_controller", "--param-file", effort_controller_config, "--controller-type", "effort_controllers/JointGroupPositionController"],
		output="screen",
	)
	spawn_joint_state_broadcaster = launch_ros.actions.Node(
		package="controller_manager",
		executable="spawner.py",
		arguments=["joint_state_broadcaster", "--controller-type", "joint_state_broadcaster/JointStateBroadcaster"],
		output="screen",
	)
	gazebo_model_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[default_model_path])

	launch_description = launch.LaunchDescription([
		gazebo_model_path,
		launch.actions.DeclareLaunchArgument(
			'use_sim_time',
			default_value='false',
			description='Use simulation (Gazebo) clock if true'),
		launch.actions.DeclareLaunchArgument(name='model', default_value=default_urdf_path,
											 description='Absolute path to robot urdf file'),
		spawn_joint_state_broadcaster,
		robot_state_publisher_node,
		gzclient,
		gzserver,
		spawn_entity,
		spawn_controller
	])

	return launch_description