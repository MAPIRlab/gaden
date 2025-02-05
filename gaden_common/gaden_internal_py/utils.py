import os
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch.frontend.parse_substitution import parse_substitution
import yaml

# Loads the custom Gaden simulation YAML and creates a LaunchConfiguration for each key
# You can use ROS substitutions inside of the simulation YAML, too
# However, it is a bad idea to use a substitution with a configuration that is defined in that same file, as the order in which the yaml keys are loaded is not guaranteed
def read_sim_yaml_path(context, path):
    with open(path) as yaml_stream:
        sim_description = yaml.load(yaml_stream, Loader=yaml.FullLoader)
        for key in sim_description:
            SetLaunchConfiguration(name=key, value=parse_substitution(str(sim_description[key]))).execute(context)

def read_sim_yaml(context):
    read_sim_yaml_path(context, os.path.join(LaunchConfiguration("pkg_dir").perform(context), 
                           "scenarios",
                            LaunchConfiguration("scenario").perform(context),
                            "simulations", 
                            LaunchConfiguration("simulation").perform(context)+".yaml"))
