# [GADEN]()

## A 3D Gas Dispersion Simulator for Mobile Robot Olfaction in Realistic Environments

GADEN is a simulation framework designed for mobile robotics systems and gas sensing algorithms, also known as Mobile Robotics Olfaction (MRO). The framework is rooted in the principles of computational fluid dynamics and filament dispersion theory, modeling wind flow and gas dispersion in 3D scenarios and accounting for walls, furniture, and other objects that may have a significative impact on the gas dispersion.

Moreover, it integrates the simulation of different environmental sensors, such as metal oxide gas sensors, photo-ionization detectors, T-DLAS or anemometers, as well as it is fully integrated with ROS and the navigation stack, making testing and validation much easier.

A demonstrative video can be seen at the [MAPIRlab channel on YouTube](https://www.youtube.com/watch?v=ZPGtk8KLtiE&ab_channel=MAPIRUMA).

Installation instructions and Tutorial for setting up new environments can be found at [GADEN_tutorial](https://github.com/MAPIRlab/gaden/blob/ros2/GADEN_tutorial.md))

Users who want to test their algorithms in complex environments but do not need to simulate any specific scenario can find an extensive repository of existing simulations in the [VGR dataset](https://mapir.isa.uma.es/mapirwebsite/?p=1708), which features simulations in 3D models of real houses, along with the configuration files and intermediate data.

## Supported ROS versions
Gaden 2 (this branch) supports only Humble, for now. You will likely run into compilation issues when using different versions. When a new ROS LTS is released we will probably migrate the project to it.

## test_env

Along with GADEN we also include in this repo a set of scenarios to help researchers to test and validate GADEN as well as their own algorithms (Gas Distribution Mapping, Gas Source Localization, etc.) in an easy way. To that end, the folder **test_env** contains multiple scenarios with pre-configured CAD models, wind-flow simulations and ROS-launch files that enable the user to easily start testing GADEN as well as their robotics solutions.

Each scenario contains the following directories:

* **cad_models**:
This folder contains all the cad models that define the environment. For convenience, we include not only walls and objects, but also the "inner" volume, that is, the free space region where gas is released (useful for CFD).

* **wind_simulations**:
All the scenarios include at least the results of one CFD simulation (see [GADEN_tutorial](https://github.com/MAPIRlab/gaden/blob/ros2/GADEN_tutorial.md)) to enable the user to directly start testing the scenario without having to worry about CFD. Notice that the repository only includes the "wind_at_cell_centers.csv" files, that is, the results of CFD after exporting the wind vectors to a CSV file. It is then necessary to run "GADEN_preprocessing.launch" to re-format this raw data to the format required by GADEN.

* **simulations**:
YAML files that describe a specific gas dispersion simulation. They specify a source location, which of the available wind simulations to use, as well as other environmental and control parameters (temperature, pressure, length of the simulation, *etc.*). These files allow you to easily switch between different simulations by using a human-readable name, rather than constantly having to modify the launch file to specify the source position and other parameters.  

* **params**:
Paramaters for the launch files that are common to all simulations in that scenario, specified using ROS-style YAML. If you are using the scenarios included here, you will probably not need to modify these at all.
 
Other than the scenarios, you will encounter the following directories:
* **launch**:
This folder includes the launch files for testing GADEN. Concretely, the files included are:

    1. **gaden_preproc_launch.py**: A launch file in charge of loading the CAD models (.stl) and CFD wind data to generate the Occupancy3D grid map and format the wind flows to the cubic-grid format used in GADEN. This must be the first launch file to run as it is a pre-processing stage necessary for the simulation.

    2. **gaden_sim_launch.py**: This launch file is the one responsible for running the gas dispersal simulation, saving the results to the designated folder (see params). This script represents the core of GADEN.

    3. **gaden_player_launch.py**: This launch file is in charge of "playing back" a simulation by just reading the results from the previous phase and also providing different simulated gas/wind sensors. Its main utility is to visualize (RViz) the gas dispersal as a point cloud and to make the gas and wind data available in the ROS architecture to other packages.

    4. **main_simbot_launch.py**: To illustrate the integration with ROS and *nav2*, this launch file set up the scenario, plays back the gas dispersal and introduces a mobile robot (equipped with different sensors) able to autonomous navigate the environment. This script can be used as a starting point for testing advanced MRO algorithms.

You can pass any scenario/simulation pair you want to these files, like this:

```
ros2 launch test_env gaden_sim_launch.py scenario:=Exp_C simulation:=sim1
```

The idea is to make it so that you never need to duplicate the launch files only to modify some data, as the launch only specifies which nodes to run, and the data is contained in separate, simpler YAML files.

* **navigation_config**:
Contains all the nav2 configuration files used by `main_simbot_launch.py`, alongside some additional resources, like a robot URDF.

## Dependencies

Although GADEN is a self-contained pkg, the also included "simulated_sensor_pkgs" (anemometer and gas sensors) depends on an external pkg defining some "olfaction" related msgs. This pkg is available in a different repository [olfaction_msgs](https://github.com/MAPIRlab/olfaction_msgs)
