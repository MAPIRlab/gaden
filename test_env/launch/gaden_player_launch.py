"""
    Launch file to run GADEN_player, a node in charge of playing back 
    the gas dispersal generated with GADEN, offering visualization, and
    simulated sensors to access the gas concentration and wind vector.

    Parameters:
        @param scenario - The scenario where dispersal takes place
        @param simulation - The wind flow actuating in the scenario
        @param source_(xyz) - The 3D position of the release point
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ============================================================================================
    # If you are launching this file directly, use these variables to choose the simulation to play back
    # if instead you are including this file in another launchfile, you can choose to override any of these values from there as LaunchConfigurations
    default_scenario = "Exp_C"
    default_simulation = "1,2,3-5,6"
    default_source_x = "2.50"  # set 2 decimals
    default_source_y = "8.50"  # set 2 decimals
    default_source_z = "0.50"  # set 2 decimals
    # ============================================================================================

    # Get the pkg directory
    pkg_dir = get_package_share_directory("test_env")

    # Params file
    params_yaml_file = os.path.join(
        pkg_dir, "scenarios", default_scenario, "params", "gaden_params.yaml"
    )

    logger = LaunchConfiguration("log_level")
    return LaunchDescription(
        [
            # Set env var to print messages to stdout immediately
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            # Declare Arguments
            # ==================
            DeclareLaunchArgument(
                "log_level",
                default_value=["info"],  # debug, info
                description="Logging level",
            ),
            DeclareLaunchArgument(
                "pkg_dir",
                default_value=pkg_dir,
                description="full path to this pkg",
            ),
            DeclareLaunchArgument(
                "scenario",
                default_value=[default_scenario],
                description="scenario to load",
            ),
            DeclareLaunchArgument(
                "simulation",
                default_value=[default_simulation],
                description="wind vector-field to load",
            ),
            DeclareLaunchArgument(
                "source_location_x",
                default_value=[default_source_x],
                description="pose.x of the source",
            ),
            DeclareLaunchArgument(
                "source_location_y",
                default_value=[default_source_y],
                description="pose.y of the source",
            ),
            DeclareLaunchArgument(
                "source_location_z",
                default_value=[default_source_z],
                description="pose.z of the source",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value=["True"],
                description="",
            ),
            # ========
            # NODES
            # ========
            # RVIZ
            Node(
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                prefix="xterm -hold -e",
                arguments=[
                    "-d" + os.path.join(pkg_dir, "navigation_config", "gaden.rviz")
                ],
                remappings=[
                    ("/initialpose", "/PioneerP3DX/initialpose"),
                    ("/goal_pose", "/PioneerP3DX/goal_pose"),
                ],
            ),
            # gaden_environment (for RVIZ visualization)
            Node(
                package="gaden_environment",
                executable="environment",
                name="gaden_environment",
                output="screen",
                parameters=[ParameterFile(params_yaml_file, allow_substs=True)],
            ),
            # gaden_player
            Node(
                package="gaden_player",
                executable="player",
                name="gaden_player",
                output="screen",
                parameters=[ParameterFile(params_yaml_file, allow_substs=True)],
            ),
        ]
    )  # end LaunchDescription
