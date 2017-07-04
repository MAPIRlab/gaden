# GADEN
A 3D Gas Dispersion Simulator for Mobile Robot Olfaction in Realistic Environments

This ROS pkg presents a simulation framework to enable the validation of robotics systems and gas sensing algorithms 
under realistic environments. The framework is rooted in the principles of computational fluid dynamics and filament 
dispersion theory, modeling wind flow and gas dispersion in 3D real-world scenarios (i.e. accounting for walls, furniture, etc.). 
Moreover, it integrates the simulation of different environmental sensors, such as metal oxide gas sensors, photo ionization 
detectors, or anemometers. For further information, please see the prject webpage at [http://mapir.isa.uma.es/work/gaden](http://mapir.isa.uma.es/work/gaden)

# DEMO
To enable a fast testing we include a "gaden_demo" pkg containing the necessary files to run a 3D demo. Concretelly, we have set two different demo.launch to test both, the simulator and the player pkgs. Notice that the player pkg requires to first execute the filament demo in order to have some data to plot, so we recomend to test both demos in the given oder:

1. gaden_simulator_demo.launch
In this demo we simulate withing a 3D environment a single gas source at a given location. We can visualize (using Rviz) the filaments dispersing in the environment, and save the results for its later processing with the gaden_player pkg. All parameters have been commented in order to be self explained.

2. gaden_player_demo.launch
After the simulation of the gas dispersal is done, we can run the gaden_player pkg in order to visualize the gas dispersion (we use a point cloud proportional to the gas concentration at each cell of the 3D environment). We also provide examples for some  gas sensors and an anemometer, which will provide the readings (visulized for example with rqt_plot tool).
