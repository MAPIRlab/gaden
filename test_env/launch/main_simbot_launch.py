import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetLaunchConfiguration,IncludeLaunchDescription,SetEnvironmentVariable,OpaqueFunction,GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.frontend.parse_substitution import parse_substitution
import xacro

#===========================
def launch_arguments():
    return [
        DeclareLaunchArgument("scenario", default_value="Exp_C"),
        DeclareLaunchArgument("simulation", default_value="sim1"),
        DeclareLaunchArgument("namespace", default_value="PioneerP3DX"),
        DeclareLaunchArgument("robot_simulator", default_value="BasicSim"),
    ]
#==========================

def launch_setup(context, *args, **kwargs):
    share_dir = get_package_share_directory("test_env")
    namespace = LaunchConfiguration("namespace").perform(context)

    # robot description for state_publisher
    robot_desc = xacro.process_file(
        os.path.join(share_dir, "navigation_config", "resources", "giraff.xacro"),
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
    
    robot_simulator = []
    simulator_mode = str(LaunchConfiguration("robot_simulator").perform(context))
    if  simulator_mode == "Coppelia":
        robot_simulator = [
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
    elif simulator_mode == "BasicSim":
        robot_simulator = [
            Node(
                package="basic_sim",
                executable="basic_sim",
                prefix = "xterm -hold -e",
                parameters=[
                    {"deltaTime": 0.1},
                    {"speed": 1.0},
                    {"worldFile": os.path.join(share_dir, "scenarios", LaunchConfiguration("scenario").perform(context), "BasicSimScene.yaml")}
                    ],
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


    anemometer = [
        Node(
            package="simulated_anemometer",
            executable="simulated_anemometer",
            name="fake_anemometer",
            parameters=[
                {"sensor_frame" : parse_substitution("$(var namespace)_anemometer_frame") },
                {"fixed_frame" : "map"},
                {"noise_std" : 0.3},
                {"use_map_ref_system" : False},
                {'use_sim_time': True},
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='anemometer_tf_pub',
            arguments = ['0', '0', '0.5', '1.0', '0.0', '0', '0', parse_substitution('$(var namespace)_base_link'), parse_substitution('$(var namespace)_anemometer_frame')],
            parameters=[{'use_sim_time': True}]
        ),
    ]

    PID = [
        Node(
            package="simulated_gas_sensor",
            executable="simulated_gas_sensor",
            name="fake_pid",
            parameters=[
                {"sensor_model" : 30 },
                {"sensor_frame" : parse_substitution("$(var namespace)_pid_frame") },
                {"fixed_frame" : "map"},
                {"noise_std" : 20.1},
                {'use_sim_time': True},
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='pid_tf_pub',
            arguments = ['0', '0', '0.5', '1.0', '0.0', '0', '0', parse_substitution('$(var namespace)_base_link'), parse_substitution('$(var namespace)_pid_frame')],
            parameters=[{'use_sim_time': True}]
        ),
    ]

    namespaced_actions = [PushRosNamespace(namespace)]
    namespaced_actions.extend(visualization_nodes)
    
    other_actions = [gaden_player]
    other_actions.extend(robot_simulator)
    other_actions.extend(anemometer)
    other_actions.extend(PID)
    other_actions.append(nav2_nodes)
    return [GroupAction(actions=namespaced_actions), 
            GroupAction(actions=other_actions)
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