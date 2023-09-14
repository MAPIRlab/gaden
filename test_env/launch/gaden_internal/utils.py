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
        SetLaunchConfiguration(name="sim_time", value=str(sim_description["sim_time"])).execute(context)
        SetLaunchConfiguration(name="time_step", value=str(sim_description["time_step"])).execute(context)
        SetLaunchConfiguration(name="num_filaments_sec", value=str(sim_description["num_filaments_sec"])).execute(context)
        SetLaunchConfiguration(name="variable_rate", value=str(sim_description["variable_rate"])).execute(context)
        SetLaunchConfiguration(name="filament_stop_steps", value=str(sim_description["filament_stop_steps"])).execute(context)
        SetLaunchConfiguration(name="ppm_filament_center", value=str(sim_description["ppm_filament_center"])).execute(context)
        SetLaunchConfiguration(name="filament_initial_std", value=str(sim_description["filament_initial_std"])).execute(context)
        SetLaunchConfiguration(name="filament_growth_gamma", value=str(sim_description["filament_growth_gamma"])).execute(context)
        SetLaunchConfiguration(name="filament_noise_std", value=str(sim_description["filament_noise_std"])).execute(context)
        SetLaunchConfiguration(name="gas_type", value=str(sim_description["gas_type"])).execute(context)
        SetLaunchConfiguration(name="temperature", value=str(sim_description["temperature"])).execute(context)