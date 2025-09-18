from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_prefix, get_package_share_directory

import launch_ros
import os

from os import environ, pathsep

from glob import glob

def get_model_paths(packages_names):
	model_paths = ""
	for package_name in packages_names:
		if model_paths != "":
			model_paths += pathsep

		package_path = get_package_prefix(package_name)
		model_path = os.path.join(package_path, "share")

		model_paths += model_path

	if "GAZEBO_MODEL_PATH" in environ:
		model_paths += pathsep + environ["GAZEBO_MODEL_PATH"]

	return model_paths

def generate_launch_description():
	
	launch_description = launch.LaunchDescription()

	moveit_config = MoveItConfigsBuilder("NEXTAGE", package_name="nextage_fillie_open_moveit_config").to_moveit_configs()

	# use_sim_time = LaunchConfiguration('use_sim_time', default='false')

	gaz_res_path = ""
	for path in glob("/usr/share/gazebo*"):
		gaz_res_path = gaz_res_path + pathsep + path


	gazebo_res_path_env_var = SetEnvironmentVariable("GAZEBO_RESOURCE_PATH", gaz_res_path)	

	launch_description.add_action(gazebo_res_path_env_var)
	
	packages = ["nextage_fillie_open_description"]

	model_path = get_model_paths(packages)
	
	gazebo_model_path_env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

	launch_description.add_action(gazebo_model_path_env_var)


	launch_description.add_action(launch.actions.DeclareLaunchArgument(
			'use_sim_time',
			default_value='True',
			description='Use simulation (Gazebo) clock if true'),
	)



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
	
	launch_description.add_action(gzclient)
	launch_description.add_action(gzserver)

	launch_description.add_action(
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				str(moveit_config.package_path / "launch/move_group.launch.py")
			),
		)
	)

	spawn_entity = Node(
		package="gazebo_ros",
		executable="spawn_entity.py",
		arguments=["-topic", "robot_description", "-entity", "Nextage",
			"-x", "0.0", "-y", "0.0", "-z", "0.8",
		],
		output="screen",
	)
	launch_description.add_action(spawn_entity)
	
	launch_description.add_action(IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				str(moveit_config.package_path / "launch/rsp.launch.py")
			),
		)
	)


	# Run Rviz and load the default config to see the state of the move_group node
	launch_description.add_action(
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
			)
		)
	)

	# Fake joint driver
	launch_description.add_action(
		Node(
			package="controller_manager",
			executable="ros2_control_node",
			parameters=[
				moveit_config.robot_description,
				str(moveit_config.package_path / "config/ros2_controllers.yaml"),
				],
		)
	)

	launch_description.add_action(
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource(
				str(moveit_config.package_path / "launch/spawn_controllers.launch.py")
			),
		)
	)



	return launch_description