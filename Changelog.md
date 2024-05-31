## 2.5.0
### Major changes
- Added support for [BasicSim](https://github.com/PepeOjeda/BasicSim). The preprocessing node now automatically generates a valid BasicSim scene yaml, and the main_simbot launchfile has an argument to let you select BasicSim as the robotic simulator.

### Minor changes
- Improved CMakelists build mode configuration and changed some of the fatal errors in `gaden_player` to raise `SIGTRAP` instead of calling `exit()` for easier debugging

## 2.4.0
### Major changes
- Simulation YAML files in the `test_env` directory are now more powerful than ever! You can define any ROS argument per-simulation, so a configuration does not need to be shared between all simulations in a given scenario. However, if you want to add a new parameter to the simulation YAML, you will also need to modify the shared YAML (under the `params` directory) to tell it to use the value that was read from the simulation file -- like this: 
```yaml 
    parameter_name: $(var parameter_name)
``` 
- CAD models for `preprocessing` and `environment` can now (at last!) be specified as a single list, rather than as disconnected, numbered parameters. So this:
```yaml
    model_0: ".../cad_models/10x6_walls.stl"
    model_1: ".../cad_models/10x6_maze_obj_1.stl"
    model_2: ".../cad_models/10x6_maze_obj_2.stl"
```
Can be replaced by:
```yaml
    models: 
      - ".../cad_models/10x6_walls.stl"
      - ".../cad_models/10x6_maze_obj_1.stl"
      - ".../cad_models/10x6_maze_obj_2.stl"
```
Which makes making modifications after the fact far more convenient. The old format is still correctly parsed, but it is considered deprecated, and will produce a warning.

### Minor changes
- A fair bit of the internal structure of gaden has been rearranged (mostly due to the gaden_common folder, which has been promoted to an ament package). This should not affect you as a user, other than maybe needing to update the repo's submodules, which have been moved as a consequence.
- Slightly modified logging from all nodes (including pretty colors, if your terminal supports them!)

### Bug fixes
- Updated CMakeLists.txt for the preprocessing package to use the correct filename for `configureCoppeliaSim`.


## 2.3.1
### Minor changes
- Added the option to use `configureCoppeliaSim` to set the robot pose at runtime by sending a pose through a topic. By default it subscribes to the `/initial_pose` topic so it can be controlled directly through Rviz.

### Bug Fixes
- Prevented anemometer speed reading from becoming negative due to sensor noise greater than the actual reading.


## 2.3.0
### Major changes
- Changed the internal format of simulation-result logfiles. `Gaden_player` still supports the old format, so existing simulations can still be played back, **but** it is strongly recommended to re-run any simulations you have to update them to the new format. The logfiles generated with the last few versions of Gaden had a bug which will now cause problems.
- Removed support for the legacy ASCII logfiles. This almost certainly does not affect you, since these files have not been a thing for years now.
- Renamed `moveCoppeliaRobot` node (under `preprocessing`) to `configureCoppeliaSim`, since it now has a more general role.
- `configureCoppeliaSim` can now control the speed of the coppelia simulation, set with a ros parameter.

### Minor changes
- Added `frequency` parameter to `fake_anemometer` to control how often it produces a new measurement .
- Added `OccupancyGridToSTL` to the `utils` directory. Pretty self-explanatory.
- Added support for wind files exported from new paraview versions, which might order the columns differently.

## 2.2.1

### Bug fixes
- Fixed an issue with the coppelia scene resetting halfway through the automatic preprocessing changes.
- Fixed an issue with the number of models being uninitialized in **preprocessing** and **environment**.

## 2.2.0

### Major changes
- Gas sensor and anemometer nodes now use <node_name> as the prefix for their published topics (*e.g.* `/Anemometer1/WindSensor_reading`). This allows you to more easily have multiple nodes of the same type, each publishing to its own topics.
- Added a node to **gaden_preprocessing** to set the initial position of the robot in the coppelia scene. Can be used a single time to permanently change the position and serialize the change, or launched every time you start the simulation to set the position for that run only.

### Minor changes
- Removed outdated **gaden_gui** package. Maybe it will be replaced with a working version in the future. Maybe.
- Changed the error message when giving an incorrect path to the **gaden_player** node. Now there is a single error telling you the folder does not exist, and then the node exits, rather than constantly spamming that each individual iteration is missing.

### Bug fixes
- Fixed a bug causing the execution rate of the simulated gas sensor to be unitialized.

## 2.1.0

### Major changes:
- Filament simulator no longer appends a long description of the simulation (`"FilamentSimulator_gas_type_source_location..."`) to the specified path for the results folder. This can break existing launch files, as the **filament_simulator** node and the gaden_player node used to require different parameters to account for this extra subfolder. 
</br>While breaking changes are obviously undesirable, the fact that from now on both nodes will expect the same string (the actual path of the results folder) should make things far less confusing.

### Minor changes
- Removed the "number of models" parameter from **environment** and **preprocessing**. The total count is now inferred from the indices of the existing parameters.
- Improved console logging from preprocessing.
- Changed the structure of test_env to include the "simulation" yamls. This will be explained in more detail in the tutorial.

## 2.0.0
### Ported Gaden to ROS2!