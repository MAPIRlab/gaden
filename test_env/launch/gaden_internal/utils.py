import os
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import yaml


def read_sim_yaml(context):
    with open(os.path.join(get_package_share_directory("test_env"), 
                           "scenarios",
                           	LaunchConfiguration("scenario").perform(context),
							"simulations", 
							LaunchConfiguration("simulation").perform(context)+".yaml")) as yaml_stream:
        sim_description = yaml.load(yaml_stream, Loader=yaml.FullLoader)
        SetLaunchConfiguration(name="wind_sim_path", value=sim_description["wind_sim_path"]).execute(context)
        SetLaunchConfiguration(name="source_x", value=str(sim_description["source_x"])).execute(context)
        SetLaunchConfiguration(name="source_y", value=str(sim_description["source_y"])).execute(context)
        SetLaunchConfiguration(name="source_z", value=str(sim_description["source_z"])).execute(context)
        SetLaunchConfiguration(name="gas_type", value=str(sim_description["gas_type"])).execute(context)