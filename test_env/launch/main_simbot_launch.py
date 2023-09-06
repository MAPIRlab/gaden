import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    OpaqueFunction,
    GroupAction,
)
from launch.launch_description_sources import (
    FrontendLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory
import xacro

# ============================================================================================
default_scenario = "Exp_C"
default_simulation = "1,2,3-5,6"
default_source_x = "2.50"  # set 2 decimals
default_source_y = "8.50"  # set 2 decimals
default_source_z = "0.50"  # set 2 decimals
use_rviz = "True"
# ============================================================================================


def launch_setup(context, *args, **kwargs):
    my_dir = get_package_share_directory("test_env")
    namespace = LaunchConfiguration("namespace").perform(context)

    # robot description for state_pùblisher
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
                        default_scenario,
                        "coppeliaScene.ttt",
                    ]
                ),
                "coppelia_headless": "False",
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
            "use_rviz": use_rviz,
            "scenario": default_scenario,
            "simulation": default_simulation,
            "source_location_x": default_source_x,
            "source_location_y": default_source_y,
            "source_location_z": default_source_z,
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
            "scenario": default_scenario,
        }.items(),
    )

    actions = [PushRosNamespace(namespace)]
    actions.extend(visualization_nodes)
    actions.extend(coppelia_launch)
    return [GroupAction(actions=actions), nav2_nodes, gaden_player]


def generate_launch_description():
    my_dir = get_package_share_directory("test_env")

    return LaunchDescription(
        [
            # Set env var to print messages to stdout immediately
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            DeclareLaunchArgument("log_level",
                default_value=["info"],  # debug, info
                description="Logging level",
            ),
            DeclareLaunchArgument("namespace", default_value="PioneerP3DX"),
            DeclareLaunchArgument("scenario", default_value=default_scenario),
            DeclareLaunchArgument("nav_params_yaml",
                default_value=os.path.join(
                    my_dir, "navigation_config", "nav2_params.yaml"
                ),
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
