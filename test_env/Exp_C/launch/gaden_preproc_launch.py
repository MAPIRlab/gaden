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
    
    # Get the launch directory
    pkg_dir = get_package_share_directory('test_env')

    # Config files
    params_yaml_file = os.path.join(pkg_dir, 'Exp_C', 'launch', 'gaden_preproc_params.yaml')
    

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
            "scenario", default_value=["Exp_C"],
            description="scenario to load",
            ),       

        # NODES
        #==========

        # gaden_preprocessing
        Node(
            package='gaden_preprocessing',
            executable='preprocessing',
            name='gaden_preprocessing',
            output='screen',
            prefix='',
            parameters=[ParameterFile(params_yaml_file, allow_substs=True)]
            )
    ]) #end LaunchDescription
