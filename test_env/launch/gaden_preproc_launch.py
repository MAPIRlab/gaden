"""
    Launch file to run the GADEN-preprocessing node.
    This is mandatory before running GADEN simulator.
    From the CAD models in the selected scenario, it generates a 3D and 2D occupancy gridmap
    that will be employed by GADEN and nav2 to simulate dispersion and navigation. Moreover parses
    the 3D cloud of wind-vectors obtained from CFD, to a grid-based format.
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from ament_index_python.packages import get_package_share_directory

# DEFAULT PARAM VALUES #
# ======================#
default_scenario = "Exp_C"
default_simulation = "1,2,3-5,6"
default_generateCoppeliaScene = False
# ======================#


def launch_setup(context, *args, **kwargs):
    scenario = LaunchConfiguration("scenario").perform(context)
    generateCoppeliaScene = (
        LaunchConfiguration("generateCoppeliaScene").perform(context) == "True"
    )
    pkg_dir = LaunchConfiguration("pkg_dir").perform(context)

    params_yaml_file = os.path.join(
        pkg_dir, "scenarios", scenario, "params", "preproc_params.yaml"
    )

    returnList = [
        Node(
            package="gaden_preprocessing",
            executable="preprocessing",
            name="gaden_preprocessing",
            output="screen",
            prefix="",
            parameters=[
                ParameterFile(params_yaml_file, allow_substs=True),
                {"generateCoppeliaScene": generateCoppeliaScene},
            ],
        )
    ]

    if generateCoppeliaScene:
        returnList.append(
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
                            pkg_dir,
                            "navigation_config",
                            "resources",
                            "default_coppelia_scene.ttt",
                        ]
                    ),
                    "coppelia_headless": "True",
                    "autoplay": "False",
                }.items(),
            )
        )

    return returnList


def generate_launch_description():
    return LaunchDescription(
        [
            # Set env var to print messages to stdout immediately
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            DeclareLaunchArgument(
                "log_level",
                default_value=["debug"],  # debug, info
                description="Logging level",
            ),
            # Declare Arguments
            # ==================
            DeclareLaunchArgument(
                "log_level",
                default_value=["info"],  # debug, info
                description="Logging level",
            ),
            DeclareLaunchArgument(
                "scenario",
                default_value=[default_scenario],
                description="scenario to preprocess",
            ),
            DeclareLaunchArgument(
                "simulation",
                default_value=[default_simulation],
                description="wind vector-field to load",
            ),
            DeclareLaunchArgument(
                "generateCoppeliaScene",
                default_value=[str(default_generateCoppeliaScene)],
                description="(bool) wether to generate a coppelia scene from this environment. See the tutorial for requirements",
            ),
            DeclareLaunchArgument(
                "pkg_dir",
                default_value=[get_package_share_directory("test_env")],
                description="full path to this pkg",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
