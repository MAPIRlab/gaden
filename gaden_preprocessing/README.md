# GADEN_preprocessing

This pkg preprocess the data contained in the *test_env* pkg to make it usable by GADEN simulator.

The data in the *test_env* pkg (a set of test scenarios) only contains the 3D CAD models of the environment and a set of 3D clouds of wind vectors generated from CFD, and exported to CSV file. This is to make this repository as light as possible. In order to simulate gas dispersal, GADEN requires this raw data to be converted to a more "robotics" format, that is, into 3D cubic grids, and this is what the *GADEN_preprocessing* pkg is about. In a nutshell, the pkg is in charge of:

1. Generating 3D and 2D occupancy gridmaps from the provided CAD models. These occupancy gridmaps are used both by GADEN for estimating the gas dispersal, as well as for the *nav2* stack to allow autonomous navigation (localization, path planning, etc.).

2. Parsing the 3D cloud of wind vectors comming from CFD, into a 3D cubic grid of wind vectors, in accordance with the 3D occupancy gridmap.

For more information, refer to the [GADEN_tutorial](https://github.com/MAPIRlab/gaden/blob/master/GADEN_tutorial.pdf), section 4.
