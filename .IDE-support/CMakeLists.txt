#This file is only to set up code analysis for all gaden packages at the same time. Not used for actual building!

cmake_minimum_required(VERSION 3.5)
project(gaden)

add_subdirectory(../gaden_environment build/gaden_environment)
add_subdirectory(../gaden_filament_simulator build/gaden_filament_simulator)
add_subdirectory(../gaden_player build/gaden_player)
add_subdirectory(../gaden_preprocessing build/gaden_preprocessing)
add_subdirectory(../simulated_anemometer build/simulated_anemometer)
add_subdirectory(../simulated_gas_sensor build/simulated_gas_sensor)
add_subdirectory(../simulated_tdlas build/simulated_tdlas)