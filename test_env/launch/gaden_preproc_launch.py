"""
    Launch file to run the GADEN-preprocessing node.
    This is mandatory before running GADEN simulator.
    From the CAD models in the selected scenario, it generates a 3D and 2D occupancy gridmap
    that will be employed by GADEN and nav2 to simulate dispersion and navigation. Moreover parses
    the 3D cloud of wind-vectors obtained from CFD, to a grid-based format.
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # SET MAIN PARAMS HERE #
    #======================#
    scenario = 'Exp_C'
    simulation = '1,2,3-5,6'
    generateCoppeliaScene = False
    #======================#

    # Get the pkg directory
    pkg_dir = get_package_share_directory('test_env')

    # Params file
    params_yaml_file = os.path.join(pkg_dir, 'scenarios', scenario, 'params', 'preproc_params.yaml')

    logger = LaunchConfiguration("log_level")    
    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        # Declare Arguments 
        # ==================
        DeclareLaunchArgument(
            "log_level", default_value=["info"],  #debug, info
            description="Logging level",
            ),
        DeclareLaunchArgument(
            "pkg_dir", default_value=[pkg_dir],
            description="full path to this pkg",
            ),
        DeclareLaunchArgument(
            "scenario", default_value=[scenario],
            description="scenario to preprocess",
            ),
        DeclareLaunchArgument(
            "simulation", default_value=[simulation],
            description="wind vector-field to load",
            ),

        #==========
        # NODES
        #==========

        # gaden_preprocessing
        Node(
            package='gaden_preprocessing',
            executable='preprocessing',
            name='gaden_preprocessing',
            output='screen',
            prefix='',
            parameters=[
                ParameterFile(params_yaml_file, allow_substs=True),
                {'generateCoppeliaScene' : generateCoppeliaScene}
                ]
            ),

        IncludeLaunchDescription(
            FrontendLaunchDescriptionSource(
                os.path.join(get_package_share_directory('coppelia_ros2_pkg'),
                    'launch/coppeliaSim.launch')
            ),
            condition=IfCondition( PythonExpression([str(generateCoppeliaScene)]) ),
            launch_arguments={
                'coppelia_scene_path': PathJoinSubstitution([
                    get_package_share_directory('test_env'),
                    'navigation_config',
                    'default_coppelia_scene.ttt'
                ]),
                'coppelia_headless': 'True',
            }.items()
        )
    ]) #end LaunchDescription
