import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetLaunchConfiguration,IncludeLaunchDescription,SetEnvironmentVariable,OpaqueFunction,GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import xacro

#===========================
def launch_arguments():
    return [
		DeclareLaunchArgument("scenario", default_value="Exp_C"),
		DeclareLaunchArgument("simulation", default_value="sim1"),
        DeclareLaunchArgument("namespace", default_value="PioneerP3DX"),
    ]
#==========================

def launch_setup(context, *args, **kwargs):
    my_dir = get_package_share_directory("test_env")
    namespace = LaunchConfiguration("namespace").perform(context)

    # robot description for state_publisher
    robot_desc = xacro.process_file(
        os.path.join(my_dir, "navigation_config", "resources", "giraff.xacro"),
        mappings={"frame_ns": namespace},
    )
    robot_desc = robot_desc.toprettyxml(indent="  ")

    visualization_nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"use_sim_time": True, "robot_description": robot_desc}],
        ),
    ]

    coppelia_launch = [
        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("coppelia_ros2_pkg"),
                    "launch/coppeliaSim.launch",
                )
            ),
            launch_arguments={
                "coppelia_scene_path": PathJoinSubstitution(
                    [
                        get_package_share_directory("test_env"),
                        "scenarios",
                        LaunchConfiguration("scenario").perform(context),
                        "coppeliaScene.ttt",
                    ]
                ),
                "coppelia_headless": "True",
                "autoplay": "True",
            }.items(),
        )
    ]

    gaden_player = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("test_env"),
                    "launch",
                    "gaden_player_launch.py",
                )
            ]
        ),
        launch_arguments={
            "use_rviz": "True",
            "scenario": LaunchConfiguration("scenario").perform(context),
            "simulation": LaunchConfiguration("simulation").perform(context)
        }.items(),
    )

    nav2_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("test_env"),
                    "navigation_config",
                    "nav2_launch.py",
                )
            ]
        ),
        launch_arguments={
            "namespace": LaunchConfiguration("namespace").perform(context),
            "scenario": LaunchConfiguration("scenario").perform(context),
        }.items(),
    )

    namespaced_actions = [PushRosNamespace(namespace)]
    namespaced_actions.extend(visualization_nodes)
    namespaced_actions.extend(coppelia_launch)
    return [GroupAction(actions=namespaced_actions), 
            nav2_nodes, gaden_player
		]


def generate_launch_description():

    launch_description = [
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        SetLaunchConfiguration(
            name="pkg_dir",
            value=[get_package_share_directory("test_env")],
        ),
        SetLaunchConfiguration(
            name="nav_params_yaml",
            value=[PathJoinSubstitution(
				[LaunchConfiguration("pkg_dir"), "navigation_config", "nav2_params.yaml"]
			)],
        ),
    ]
    
    launch_description.extend(launch_arguments())
    launch_description.append(OpaqueFunction(function=launch_setup))
    
    return  LaunchDescription(launch_description)