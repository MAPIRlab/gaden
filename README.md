# [GADEN](http://mapir.isa.uma.es/work/gaden)
**A 3D Gas Dispersion Simulator for Mobile Robot Olfaction in Realistic Environments**

GADEN is a simulation framework designed for mobile robotics systems and gas sensing algorithms, also known as Mobile Robotics Olfaction (MRO). The framework is rooted in the principles of computational fluid dynamics and filament dispersion theory, modeling wind flow and gas dispersion in 3D scenarios and accounting for walls, furniture, and other objects that may have a significative impact on the gas dispersion.
Moreover, it integrates the simulation of different environmental sensors, such as metal oxide gas sensors, photo-ionization 
detectors, T-DLAS or anemometers, as well as it is fully integrated with ROS and the navigation stack, making testing and validation much easier. 

A demonstrative video can be seen at the [MAPIRlab channel on YouTube](https://www.youtube.com/watch?v=ZPGtk8KLtiE&ab_channel=MAPIRUMA).

Installation instructions and Tutorial for setting up new environments can be found at [GADEN_tutorial.pdf](https://github.com/MAPIRlab/gaden/blob/master/GADEN_tutorial.pdf))

Users who want to test their algorithms in complex environments but do not need to simulate any specific scenario can find an extensive repository of existing simulations in the [VGR dataset](https://mapir.isa.uma.es/mapirwebsite/?p=1708), which features simulations in 3D models of real houses, along with the configuration files and intermediate data. 

## TEST_ENV
Along with GADEN we also include in this repo a set of scenarios to help researchers to test and validate GADEN as well as their own algorithms (Gas Distribution Mapping, Gas Source Localization, etc.) in an easy way. To that end, the folder **test_env** contains multiple scenarios with pre-configured CAD models, wind-flow simulations and ROS-launch files that enable the user to easily start testing GADEN as well as their robotics solutions. Each scenario follows a similar structure:

* **cad_models**:
This folder contains all the cad models that define the environment. For convenience, we include not only walls and objects, but also the "inner" volume, that is, the free space region where gas is released (useful for CFD).

* **launch**:
This folder includes the launch and configuration files for testing GADEN and a simple Robotics simulation environment using [Stage](http://wiki.ros.org/stage). Concretely, the files included are:

    1. **GADEN_preprocessing**: A launch file in charge of loading the CAD models (.stl ASCII) and CFD wind data to generate the Occupancy3D grid map and format the wind flows to the cubic-grid format used in GADEN. This must be the first launch file to run as it is a pre-processing stage necessary for the simulation.

    2. **GADEN**: This launch file is the one responsible for running the gas dispersal simulation, saving the results to the designated folder (see params). This script represents the core of GADEN.

    3. **GADEN_player**: This launch file is in charge of "playing back" a simulation by just reading the results from the previous phase and also providing different simulated gas/wind sensors. Its main utility is to visualize (RViz) the gas dispersal as a point cloud and to make the gas and wind data available in the ROS architecture to other packages.

    4. **main_simbot**: To illustrate the integration with ROS and *move_base*, this launch file set up the scenario, plays back the gas dispersal and introduces a mobile robot (equipped with different sensors) able to autonomous navigate the environment. This script can be used as a starting point for testing advanced MRO algorithms.
  
* **wind_simulations**:
All the scenarios include at least the results of one CFD simulation (see [GADEN_tutorial.pdf](https://github.com/MAPIRlab/gaden/blob/master/GADEN_tutorial.pdf)) to enable the user to directly start testing the scenario without having to worry about CFD. Notice that the repository only includes the "wind_at_cell_centers.csv" files, that is, the results of CFD after exporting the wind vectors to a CSV file. It is then necessary to run "GADEN_preprocessing.launch" to re-format this raw data to the format required by GADEN.



## olfaction_msgs
Although GADEN is a self-contained pkg, the also included "simulated_sensor_pkgs" (anemometer and gas sensors) depends on an external pkg defining some "olfaction" related msgs. This pkg is available in a different repository [olfaction_msgs](https://github.com/MAPIRlab/olfaction_msgs)
