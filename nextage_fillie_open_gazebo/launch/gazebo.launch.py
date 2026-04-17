from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from moveit_configs_utils.launch_utils import add_debuggable_node, DeclareBooleanLaunchArg

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
			default_value='true',
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

	# launch_description.add_action(
	# 	IncludeLaunchDescription(
	# 		PythonLaunchDescriptionSource(
	# 			str(moveit_config.package_path / "launch/move_group.launch.py")
	# 		),
	# 	)
	# )

	launch_description.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
	launch_description.add_action(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
	launch_description.add_action(
        DeclareBooleanLaunchArg("publish_monitored_planning_scene", default_value=True)
    )
    # load non-default MoveGroup capabilities (space separated)
	launch_description.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    # inhibit these default MoveGroup capabilities (space separated)
	launch_description.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    # do not copy dynamics information from /joint_states to internal robot monitoring
    # default to false, because almost nothing in move_group relies on this information
	launch_description.add_action(DeclareBooleanLaunchArg("monitor_dynamics", default_value=False))

	should_publish = LaunchConfiguration("publish_monitored_planning_scene")

	move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": should_publish,
        "publish_geometry_updates": should_publish,
        "publish_state_updates": should_publish,
        "publish_transforms_updates": should_publish,
        "monitor_dynamics": False,
    }

	move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
		{"use_sim_time": True}
    ]

	add_debuggable_node(
        launch_description,
        package="moveit_ros_move_group",
        executable="move_group",
        commands_file=str(moveit_config.package_path / "launch" / "gdb_settings.gdb"),
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"],
        # Set the display variable, in case OpenGL code is used internally
        additional_env={"DISPLAY": ":0"},
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
	# launch_description.add_action(
	# 	IncludeLaunchDescription(
	# 		PythonLaunchDescriptionSource(
	# 			str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
	# 		)
	# 	)
	# )

	launch_description.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=str(moveit_config.package_path / "config/moveit.rviz"),
        )
    )
	rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
		{"use_sim_time": True}
    ]
	add_debuggable_node(
        launch_description,
        package="rviz2",
        executable="rviz2",
        output="log",
        respawn=False,
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=rviz_parameters,
    )

	# Fake joint driver
	launch_description.add_action(
		Node(
			package="controller_manager",
			executable="ros2_control_node",
			parameters=[
				moveit_config.robot_description,
				str(moveit_config.package_path / "config/ros2_controllers.yaml"),
				{"use_sim_time": True}
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