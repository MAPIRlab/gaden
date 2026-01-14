# GADEN

## A 3D Gas Dispersion Simulator for ROS

GADEN is a simulation framework designed for mobile robotics systems and gas sensing algorithms, also known as Mobile Robotics Olfaction (MRO). The framework is rooted in the principles of computational fluid dynamics and filament dispersion theory, modeling wind flow and gas dispersion in 3D scenarios and accounting for walls, furniture, and other objects that may have a significative impact on the gas dispersion.

Moreover, it integrates the simulation of different environmental sensors, such as metal oxide gas sensors, photo-ionization detectors, T-DLAS or anemometers, as well as it is fully integrated with ROS and the navigation stack, making testing and validation much easier.

If you do not want to use ROS, gaden is available as a [standalone library](https://github.com/MAPIRlab/gaden_core) and a [graphical desktop application](https://github.com/MAPIRlab/gaden_gui).

## Video showcase
[![Video showcase](https://img.youtube.com/vi/2i3_pyV-MYU/hqdefault.jpg)](https://youtu.be/2i3_pyV-MYU)

You can cite Gaden in your work using the following .bib:

```
@ARTICLE{gaden-rt-2025,
    author = {Ojeda, Pepe and Monroy, Javier and Gonzalez-Jimenez, Javier},
     title = {Gaden-RT: A Real Time and Interactive Gas Dispersion Simulator for Mobile Robotics},
   journal = {SoftwareX},
    volume = {32},
      year = {2025},
      issn = {2352-7110},
       url = {https://www.sciencedirect.com/science/article/pii/S2352711025003541},
       doi = {10.1016/j.softx.2025.102388}
}

@article{monroyGADEN3DGas2017,
     title = {{{GADEN}}: {{A 3D}} Gas Dispersion Simulator for Mobile Robot Olfaction in Realistic Environments},
    author = {Monroy, Javier and {Hernandez-Bennetts}, Victor and Fan, Han and Lilienthal, Achim and {Gonzalez-Jimenez}, Javier},
      year = {2017},
   journal = {Sensors (Switzerland)},
    volume = {17},
    number = {7},
     pages = {1--16},
       doi = {10.3390/s17071479},
}
```

## Installation
Move to your colcon workspace and run

`git clone --recurse-submodules git@github.com:MAPIRlab/gaden.git src/gaden`

It is important that you include the `--recurse-submodules` option, as the gaden repository includes submodules. if you already cloned in a non-recursive manner you can fix the problem by running 

`git submodule update --init --recursive`

#### Dependencies
Although GADEN is a self-contained pkg, the also included "simulated_sensor_pkgs" (anemometer and gas sensors) depends on an external pkg defining some "olfaction" related msgs. This pkg is available in a different repository [olfaction_msgs](https://github.com/MAPIRlab/olfaction_msgs)

## Usage
See the [tutorial](GADEN_tutorial.md) for instructions on how to set up a simulation of your own or use the included [test environments](test_env).

Users who want to test their algorithms in complex environments but do not need to simulate any specific scenario can find an extensive repository of existing simulations in the [VGR dataset](https://mapir.isa.uma.es/mapirwebsite/?p=1708), which features simulations in 3D models of real houses, along with the configuration files and intermediate data.
