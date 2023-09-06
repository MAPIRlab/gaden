import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
    GroupAction,
)
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import xacro


def launch_setup(context, *args, **kwargs):
    # Get the launch directory
    my_dir = get_package_share_directory("test_env")
    map_file = os.path.join(
        my_dir,
        "scenarios",
        LaunchConfiguration("scenario").perform(context),
        "occupancy.yaml",
    )
    namespace = LaunchConfiguration("namespace").perform(context)

    # common variables
    configured_params = ParameterFile(
        LaunchConfiguration("nav_params_yaml").perform(context), allow_substs=True
    )
    use_sim_time = True

    navigation_nodes = [
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"yaml_filename": map_file},
                {"frame_id": "map"},
            ],
        ),
        # BT NAV
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[configured_params],
        ),
        # PLANNER (global path planning)
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[configured_params],
        ),
        # CONTROLLER (local planner / path following)
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            parameters=[configured_params],
        ),
        # RECOVERIES (recovery behaviours NOT YET in HUMBLE)
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[configured_params],
        ),
        # LIFECYCLE MANAGER
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": True},
                {
                    "node_names": [
                        "map_server",
                        "planner_server",
                        "controller_server",
                        "bt_navigator",
                        "behavior_server",
                    ]
                },
            ],
        ),
    ]

    actions = [PushRosNamespace(namespace)]
    actions.extend(navigation_nodes)
    return [GroupAction(actions=actions)]


def generate_launch_description():
    my_dir = get_package_share_directory("test_env")

    return LaunchDescription(
        [
            # Set env var to print messages to stdout immediately
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument(
                "log_level",
                default_value=["info"],  # debug, info
                description="Logging level",
            ),
            DeclareLaunchArgument("namespace", default_value="PioneerP3DX"),
            DeclareLaunchArgument("scenario", default_value="Exp_C"),  # required
            DeclareLaunchArgument(
                "nav_params_yaml",
                default_value=os.path.join(
                    my_dir, "navigation_config", "nav2_params.yaml"
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
