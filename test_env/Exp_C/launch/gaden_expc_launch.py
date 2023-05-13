import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Variables
    pkg_dir = get_package_share_directory('test_env')
    scenario = 'Exp_C'
    simulation = '1,2,3-5,6'
    source_x = '6.5'
    source_y = '2.2'
    source_z = '0.5'

    # Param files
    params_yaml_file = os.path.join(pkg_dir, scenario, 'launch', 'gaden_expc_params.yaml')    

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
            "pkg_dir", default_value=pkg_dir,
            description="full path to this pkg",
            ),
        DeclareLaunchArgument(
            "scenario", default_value=[scenario],
            description="scenario to load",
            ),
        DeclareLaunchArgument(
            "simulation", default_value=[simulation],
            description="wind vector-field to load",
            ),
        DeclareLaunchArgument(
            "source_location_x", default_value=[source_x],
            description="pose.x of the source",
            ),
        DeclareLaunchArgument(
            "source_location_y", default_value=[source_y],
            description="pose.x of the source",
            ),
        DeclareLaunchArgument(
            "source_location_z", default_value=[source_z],
            description="pose.x of the source",
            ),
    

        # NODES
        #==========

        # RVIZ
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            prefix="xterm -hold -e",
            arguments=['-d' + os.path.join(pkg_dir, scenario, 'launch', 'ros', 'gaden.rviz')]
            ),

        # gaden_environment (for RVIZ visualization)
        Node(
            package='gaden_environment',
            executable='environment',
            name='gaden_environment',
            output='screen',
            prefix='xterm -hold -e',
            parameters=[ParameterFile(params_yaml_file, allow_substs=True)]
            ),

        # gaden_filament_simulator (The core)
        Node(
            package='gaden_filament_simulator',
            executable='filament_simulator',
            name='gaden_filament_simulator',
            output='screen',
            prefix='xterm -hold -e',
            parameters=[ParameterFile(params_yaml_file, allow_substs=True)]
            ),
        
        # MAP_SERVER
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename' : os.path.join(pkg_dir, scenario, 'occupancy.yaml')}]
            ),
        # LIFECYCLE MANAGER
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': ['map_server']}
                       ]
            )
    ]) #end LaunchDescription
