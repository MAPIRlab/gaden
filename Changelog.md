## 2.1.0

### Important changes:
- Filament simulator no longer appends a long description of the simulation ("FilamentSimulator_gas_type_source_location...") to the specified path for the results folder. This can break existing launch files, as the filament_simulator node and the gaden_player node used to require different parameters to account for this extra subfolder. 
</br>While breaking changes are obviously undesirable, the fact that from now on both nodes will expect the same string (the actual path of the results folder), should make things far less confusing.

### Minor changes
- Removed the "number of models" parameter from **environment** and **preprocessing**. The total count is now inferred from the indices of the existing parameters.
- Improved console logging from preprocessing.
- Changed the structure of test_env to include the "simulation" yamls. This will be explained in more detail in the tutorial.

## 2.0.0
### Ported Gaden to ROS2!