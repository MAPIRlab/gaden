#!/bin/bash

colcon build --symlink-install --packages-select \
gaden_common                                                             \
gaden_environment                                                        \
gaden_filament_simulator                                                 \
gaden_msgs                                                               \
gaden_player                                                             \
gaden_preprocessing                                                      \
simulated_anemometer                                                     \
simulated_gas_sensor                                                     \
simulated_tdlas                                                          \
test_env