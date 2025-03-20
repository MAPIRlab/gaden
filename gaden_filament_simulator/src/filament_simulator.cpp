/*---------------------------------------------------------------------------------------
 * MAIN Node for the simulation of gas dispersal using a Filament-based approach.
 * This node loads the wind field (usually from CFD simulation), and simulates over it
 * different filaments to spread gas particles.
 *
 * Each filament is composed of a fixed number of gas molecules (Q)
 * Each filament is determined by its center position and width.
 * The width of a filament increases over time (Turbulent and molecular difussion)
 * The position of a filament is updated with the wind.
 *
 * The gas concentration at a given point is the sum of the concentration of all filaments.
 *
 * Thus, the gas concentration at the source location is determined by the number of molecules/filament and the number of filaments.
 *
 * A log file is recorded for every snapshot (time-step) with information about the gas
 * concentration and wind vector for every cell (3D) of the environment.
 *
 * The node implements the filament-base gas dispersal simulation. At each time step, the puffs
 * of filaments are sequentially released at a source location. Each puff is composed of n filaments.
 * Filaments are affected by turbulence and molecular diffusion along its path while being transported
 * by advection with the wind. The 3-dimensional positions of these filaments are represented by the points
 * of the “visualization msgs/markers”. At each time step, “Dispersal_Simulation” node calculates or
 * determines the positions of n filaments. Gas plumes are simulated with or without acceleration.
 *
 * It is very time consuming, and currently it runs in just one thread (designed to run offline).
 *
 * TODO: Cambiar std::vector por std::list para los FILAMENTOS
 ---------------------------------------------------------------------------------------*/

#include "filament_simulator/filament_simulator.h"
#include "filament_simulator/filament.h"
#include "gaden_common/GadenVersion.h"
#include "gaden_common/ReadEnvironment.h"
#include "gaden_common/Vector3.h"
#include <filesystem>
#include <random>
#define GADEN_LOGGER_ID "FilamentSimulator"
#include <gaden_common/Logging.h>
#include <gaden_common/Time.hpp>

//==========================//
//      Constructor         //
//==========================//
CFilamentSimulator::CFilamentSimulator()
    : rclcpp::Node("Gaden_filament_simulator")
{
    // Init variables
    //-----------------
    sim_time = 0.0;                          // Start at time = 0(sec)
    sim_time_last_wind = -2 * windTime_step; // Force to load wind-data on startup
    current_wind_snapshot = 0;               // Start with wind_iter= 0;
    current_simulation_step = 0;             // Start with iter= 0;
    last_saved_step = -1;
    wind_notified = false; // To warn the user (only once) that no more wind data is found!
    wind_finished = false;
    last_saved_timestamp = -__DBL_MAX__;

    loadNodeParameters();

    // Create directory to save results (if needed)
    if (save_results && !boost::filesystem::exists(results_location))
        if (!boost::filesystem::create_directories(results_location))
            GADEN_WARN("Could not create result directory: {}", results_location.c_str());

    if (save_results && !boost::filesystem::exists(results_location + "/wind"))
        if (!boost::filesystem::create_directories(results_location + "/wind"))
            GADEN_WARN("Could not create result directory: {}/wind", results_location.c_str());

    // Set Publishers and Subscribers
    //-------------------------------
    marker_pub = create_publisher<visualization_msgs::msg::Marker>("filament_visualization", 1);

    // Wait preprocessing Node to finish?
    preprocessing_done = false;
    if (wait_preprocessing)
    {
        prepro_sub = create_subscription<std_msgs::msg::Bool>("preprocessing_done", 1,
                                                              std::bind(&CFilamentSimulator::preprocessingCB, this, std::placeholders::_1));
        while (rclcpp::ok() && !preprocessing_done)
        {
            using namespace std::literals::chrono_literals;
            rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(0.5s));
            rclcpp::spin_some(shared_from_this());
            if (verbose)
                GADEN_INFO("Waiting for node GADEN_preprocessing to end.");
        }
    }

    // Init the Simulator
    initSimulator();

    // Fluid Dynamics Eq
    /*/-----------------
     * Ideal gas equation:
     * PV = nRT
     * P is the pressure of the gas (atm)
     * V is the volume of the gas (cm^3)
     * n is the amount of substance of gas (mol) = m/M where m=mass of the gas [g] and M is the molar mass
     * R is the ideal, or universal, gas constant, equal to the product of the Boltzmann constant and the Avogadro constant. (82.057338
     * cm^3·atm/mol·k) T is the temperature of the gas (kelvin)
     */
    float R = 82.057338;                                                             //[cm³·atm/mol·K] Gas Constant
    filament_initial_vol = pow(6 * filament_initial_std, 3);                         //[cm³] -> We approximate the infinite volumen of the 3DGaussian as 6 sigmas.
    env_cell_vol = pow(environment.description.cell_size * 100, 3);                  //[cm³] Volumen of a cell
    filament_numMoles = (envPressure * filament_initial_vol) / (R * envTemperature); //[mol] Num of moles of Air in that volume
    env_cell_numMoles = (envPressure * env_cell_vol) / (R * envTemperature);         //[mol] Num of moles of Air in that volume

    // The moles of target_gas in a Filament are distributted following a 3D Gaussian
    // Given the ppm value at the center of the filament, we approximate the total number of gas moles in that filament.
    float numMoles_in_cm3 = envPressure / (R * envTemperature);                           //[mol of all gases/cm³]
    float filament_moles_cm3_center = filament_ppm_center / pow(10, 6) * numMoles_in_cm3; //[moles of target gas / cm³]
    filament_numMoles_of_gas =
        filament_moles_cm3_center * (sqrt(8 * pow(3.14159, 3)) * pow(filament_initial_std, 3)); // total number of moles in a filament

    if (verbose)
        GADEN_INFO("filament_initial_vol [cm3]: {}", filament_initial_vol);
    if (verbose)
        GADEN_INFO("env_cell_vol [cm3]: {}", env_cell_vol);
    if (verbose)
        GADEN_INFO("filament_numMoles [mol]: {}", filament_numMoles);
    if (verbose)
        GADEN_INFO("env_cell_numMoles [mol]: {}", env_cell_numMoles);
    if (verbose)
        GADEN_INFO("filament_numMoles_of_gas [mol]: {}", filament_numMoles_of_gas);

    // Init visualization
    //-------------------
    filament_marker.header.frame_id = fixed_frame;
    filament_marker.ns = "filaments";
    filament_marker.action = visualization_msgs::msg::Marker::ADD;
    filament_marker.id = 0;
    filament_marker.type = visualization_msgs::msg::Marker::POINTS;
    filament_marker.color.a = 1;
}

CFilamentSimulator::~CFilamentSimulator()
{}

//==============================//
//      GADEN_preprocessing CB  //
//==============================//
void CFilamentSimulator::preprocessingCB(const std_msgs::msg::Bool::SharedPtr b)
{
    preprocessing_done = true;
}

//==========================//
//      Load Params         //
//==========================//
void CFilamentSimulator::loadNodeParameters()
{
    // Verbose
    verbose = declare_parameter<bool>("verbose", false);

    // Wait PreProcessing
    wait_preprocessing = declare_parameter<bool>("wait_preprocessing", false);

    // Simulation Time (sec)
    max_sim_time = declare_parameter<float>("sim_time", 20.0);

    // Time increment between Gas snapshots (sec)
    time_step = declare_parameter<float>("time_step", 1.0);
    // Number of iterations to carry on = max_sim_time/time_step
    numSteps = floor(max_sim_time / time_step);

    // Num of filaments/sec
    numFilaments_sec = declare_parameter<int>("num_filaments_sec", 100);
    variable_rate = declare_parameter<bool>("variable_rate", false);
    numFilaments_step = numFilaments_sec * time_step;
    numFilament_aux = 0;
    total_number_filaments = numFilaments_step * numSteps;
    current_number_filaments = 0;

    filament_stop_steps = declare_parameter<int>("filament_stop_steps", 0);
    filament_stop_counter = 0;

    // Gas concentration at the filament center - 3D gaussian [ppm]
    filament_ppm_center = declare_parameter<float>("ppm_filament_center", 20);

    // [cm] Sigma of the filament at t=0-> 3DGaussian shape
    filament_initial_std = declare_parameter<float>("filament_initial_std", 1.5);

    // [cm²/s] Growth ratio of the filament_std
    filament_growth_gamma = declare_parameter<float>("filament_growth_gamma", 10.0);

    // [cm] Sigma of the white noise added on each iteration
    filament_noise_std = declare_parameter<float>("filament_noise_std", 0.1);

    // Gas Type ID
    gasType = declare_parameter<int>("gas_type", 1);

    // Environment temperature (necessary for molecules/cm3 -> ppm)
    envTemperature = declare_parameter<float>("temperature", 298.0);

    // Enviorment pressure (necessary for molecules/cm3 -> ppm)
    envPressure = declare_parameter<float>("pressure", 1.0);

    // Gas concentration units (0= molecules/cm3,  1=ppm)
    gasConc_unit = declare_parameter<int>("concentration_unit_choice", 1);

    // WIND DATA
    //----------
    // CFD wind files location
    wind_files_location = declare_parameter<std::string>("wind_data", "");
    //(sec) Time increment between Wind snapshots --> Determines when to load a new wind field
    windTime_step = declare_parameter<float>("wind_time_step", 1.0);
    // Loop
    allow_looping = declare_parameter<bool>("allow_looping", false);
    loop_from_step = declare_parameter<int>("loop_from_step", 1);
    loop_to_step = declare_parameter<int>("loop_to_step", 100);

    // ENVIRONMENT
    //-----------
    //  Occupancy gridmap 3D location
    occupancy3D_filepath = declare_parameter<std::string>("occupancy3D_data", "");

    // fixed frame (to disaply the gas particles on RVIZ)
    fixed_frame = declare_parameter<std::string>("fixed_frame", "map");

    // Source postion (x,y,z)
    gas_source_pos.x = declare_parameter<float>("source_position_x", 1.0);
    gas_source_pos.y = declare_parameter<float>("source_position_y", 1.0);
    gas_source_pos.z = declare_parameter<float>("source_position_z", 1.0);

    // Simulation results.
    save_results = declare_parameter<int>("save_results", 1);
    results_location = declare_parameter<std::string>("results_location", "");

    if (save_results && !boost::filesystem::exists(results_location))
    {
        if (!boost::filesystem::create_directories(results_location))
            GADEN_WARN("Could not create result directory: {}", results_location.c_str());
    }
    // create a sub-folder for this specific simulation
    results_min_time = declare_parameter<float>("results_min_time", 0.0);
    results_time_step = declare_parameter<float>("results_time_step", 1.0);

    if (verbose)
    {
        GADEN_INFO("The data provided in the roslaunch file is:");
        GADEN_INFO("Simulation Time        {}(s)", sim_time);
        GADEN_INFO("Gas Time Step:         {}(s)", time_step);
        GADEN_INFO("Num_steps:             {}", numSteps);
        GADEN_INFO("Number of filaments:   {}", numFilaments_sec);
        GADEN_INFO("PPM filament center    {}", filament_ppm_center);
        GADEN_INFO("Gas type:              {}", gasType);
        GADEN_INFO("Concentration unit:    {}", gasConc_unit);
        GADEN_INFO("Wind_time_step:        {}(s)", windTime_step);
        GADEN_INFO("Fixed frame:           {}", fixed_frame.c_str());
        GADEN_INFO("Source position:       ({},{},{})", gas_source_pos.x, gas_source_pos.y, gas_source_pos.z);

        if (save_results)
            GADEN_INFO("Saving results to {}", results_location.c_str());
    }
}

//==========================//
//                          //
//==========================//
void CFilamentSimulator::initSimulator()
{
    if (verbose)
        GADEN_INFO("Initializing Simulator... Please Wait!");

    // 1. Load Environment and Configure Matrices
    if (verbose)
        GADEN_INFO("Loading 3D Occupancy GridMap");

    gaden::ReadResult result = gaden::readEnvFile(occupancy3D_filepath, environment);
    if (result == gaden::ReadResult::NO_FILE)
    {
        GADEN_FATAL("File provided to filament-simulator node '{}' does not exist!", occupancy3D_filepath);
    }
    else if (result == gaden::ReadResult::READING_FAILED)
        GADEN_FATAL("Something went wrong while parsing the file!");

    if (verbose)
        GADEN_INFO("Env dimensions ({:.2f},{:.2f},{:.2f}) to ({:.2f},{:.2f},{:.2f})", environment.description.min_coord.x,
                   environment.description.min_coord.y, environment.description.min_coord.z, environment.description.max_coord.x,
                   environment.description.max_coord.y, environment.description.max_coord.z);
    if (verbose)
        GADEN_INFO("Env size in cells	 ({},{},{}) - with cell size {} [m]", environment.description.dimensions.x,
                   environment.description.dimensions.y, environment.description.dimensions.z, environment.description.cell_size);

    // Reserve memory for the 3D matrices: U,V,W,C and Env, according to provided num_cells of the environment.
    // It also init them to 0.0 values
    wind.resize(environment.numCells());
    environment.Env.resize(environment.numCells());

    // 2. Load the first Wind snapshot from file (all 3 components U,V,W)
    read_wind_snapshot(current_simulation_step);

    // 3. Initialize the filaments vector to its max value (to avoid increasing the size at runtime)
    if (verbose)
        GADEN_INFO("Initializing Filaments");
    filaments.resize(total_number_filaments, CFilament(0.0, 0.0, 0.0, filament_initial_std));
}

//==========================//
//                          //
//==========================//
void CFilamentSimulator::read_wind_snapshot(int idx)
{
    if (last_wind_idx == idx)
        return;

    // read wind data from disk. The old format (which we still support) used one file per component, so we need to check if this simulation has that
    // format or the new one
    if (parseWindFile(idx) || parseOldWindFiles(idx))
    {
        last_wind_idx = idx;
        if (verbose)
            GADEN_INFO("Loading Wind Snapshot {}", idx);

        if (!wind_finished)
        {
            // dump the binary wind data to file
            std::string out_filename = fmt::format("{}/wind/wind_iteration_{}", results_location, idx);
            std::ofstream outputWindFile(out_filename.c_str());

            outputWindFile.write((char*)&gaden::version_major, sizeof(int));
            outputWindFile.write((char*)&gaden::version_minor, sizeof(int));
            outputWindFile.write((char*)wind.data(), sizeof(gaden::Vector3) * wind.size());
            outputWindFile.close();
        }
    }
    else
    {
        // No more wind data. Keep current info.
        if (!wind_notified)
        {
            std::string windFile = fmt::format("{}_{}.csv_gaden", wind_files_location, idx);
            std::string separator = (wind_files_location.back() == '_') ? "" : "_";
            std::string U_filename = fmt::format("{}{}{}.csv_U", wind_files_location, separator, idx);

            GADEN_WARN("Could not find wind file in '{}' or '{}'", windFile, U_filename);
            GADEN_WARN("No more wind data available. Using last Wind snapshopt as SteadyState.");
            wind_notified = true;
            wind_finished = true;
        }
    }
}

//==========================//
//                          //
//==========================//
bool CFilamentSimulator::parseWindFile(int idx)
{
    std::string windFile = fmt::format("{}_{}.csv_gaden", wind_files_location, idx);
    if (std::filesystem::exists(windFile))
    {
        std::ifstream infile(windFile, std::ios_base::binary);

        // header
        int fileVersionMajor, fileVersionMinor;
        infile.read((char*)&fileVersionMajor, sizeof(int));
        infile.read((char*)&fileVersionMinor, sizeof(int));

        // contents
        infile.read((char*)wind.data(), sizeof(gaden::Vector3) * wind.size());
        infile.close();
        return true;
    }
    return false;
}

bool CFilamentSimulator::parseOldWindFiles(int idx)
{
    // the old-old way to do this was to pass "path/wind_" as the parameter and have this code only append the index itself
    // but that is clunky, and inconsistent with all the other gaden nodes, which all append the underscore automatically
    // so now, for backwards compatibility, we need to check whether the underscore is already there or not
    std::string separator = (wind_files_location.back() == '_') ? "" : "_";
    std::string U_filename = fmt::format("{}{}{}.csv_U", wind_files_location, separator, idx);
    std::string V_filename = fmt::format("{}{}{}.csv_V", wind_files_location, separator, idx);
    std::string W_filename = fmt::format("{}{}{}.csv_W", wind_files_location, separator, idx);

    // read data to 3D matrices
    if (std::filesystem::exists(U_filename))
    {
        if (verbose)
            GADEN_INFO("Reading Wind Snapshot {}", U_filename.c_str());

        last_wind_idx = idx;
        if (verbose)
            GADEN_INFO("Loading Wind Snapshot {}", idx);

        std::ifstream Uinfile(U_filename, std::ios_base::binary);
        std::ifstream Vinfile(V_filename, std::ios_base::binary);
        std::ifstream Winfile(W_filename, std::ios_base::binary);

        // old files always start with a single int equal to 999
        Uinfile.seekg(sizeof(int));
        Vinfile.seekg(sizeof(int));
        Winfile.seekg(sizeof(int));

        for (size_t i = 0; i < wind.size(); i++)
        {
            // old files used doubles, but we are using floats now
            double aux;
            Uinfile.read((char*)&aux, sizeof(double));
            wind[i].x = aux;
            Vinfile.read((char*)&aux, sizeof(double));
            wind[i].y = aux;
            Winfile.read((char*)&aux, sizeof(double));
            wind[i].z = aux;
        }

        Uinfile.close();
        Vinfile.close();
        Winfile.close();

        return true;
    }

    return false;
}

// Add new filaments. On each step add a total of "numFilaments_step"
void CFilamentSimulator::add_new_filaments(float radius_arround_source)
{
    numFilament_aux += numFilaments_step;
    // Release rate
    int filaments_to_release = floor(numFilament_aux);
    if (variable_rate)
    {
        filaments_to_release = (int)round(random_number(0.0, filaments_to_release));
    }
    else
    {
        if (filament_stop_counter == filament_stop_steps)
        {
            filament_stop_counter = 0;
        }
        else
        {
            filament_stop_counter++;
            filaments_to_release = 0;
        }
    }
    for (int i = 0; i < filaments_to_release; i++)
    {
        float x, y, z;
        do
        {
            // Set position of new filament within the especified radius arround the gas source location
            x = gas_source_pos.x + random_number(-1, 1) * radius_arround_source;
            y = gas_source_pos.y + random_number(-1, 1) * radius_arround_source;
            z = gas_source_pos.z + random_number(-1, 1) * radius_arround_source;
        } while (check_pose_with_environment(x, y, z) != gaden::CellState::Free);

        /*Instead of adding new filaments to the filaments vector on each iteration (push_back)
          we had initially resized the filaments vector to the max number of filaments (numSteps*numFilaments_step)
          Here we will "activate" just the corresponding filaments for this step.*/
        filaments[current_number_filaments + i].activate_filament(x, y, z, sim_time);
    }
}

//==========================//
//                          //
//==========================//

// Check if a given 3D pose falls in:
//  0 = free space
//  1 = obstacle, wall, or outside the environment
//  2 = outlet (usefull to disable filaments)
gaden::CellState CFilamentSimulator::check_pose_with_environment(float pose_x, float pose_y, float pose_z)
{
    // Get 3D cell of the point
    int x_idx = std::floor((pose_x - environment.description.min_coord.x) / environment.description.cell_size);
    int y_idx = std::floor((pose_y - environment.description.min_coord.y) / environment.description.cell_size);
    int z_idx = std::floor((pose_z - environment.description.min_coord.z) / environment.description.cell_size);

    if (x_idx < 0 || x_idx >= environment.description.dimensions.x ||
        y_idx < 0 || y_idx >= environment.description.dimensions.y ||
        z_idx < 0 || z_idx >= environment.description.dimensions.z)
        return gaden::CellState::OutOfBounds;

    // 1.2. Return cell occupancy (0=free, 1=obstacle, 2=outlet)
    return environment.at(x_idx, y_idx, z_idx);
}

//==========================//
//                          //
//==========================//
gaden::CellState CFilamentSimulator::moveFilament(CFilament& filament, float end_x, float end_y, float end_z)
{
    const bool PATH_OBSTRUCTED = true;
    const bool PATH_UNOBSTRUCTED = false;

    // Calculate displacement vector
    gaden::Vector3 end(end_x, end_y, end_z);
    gaden::Vector3 movementDir = end - filament.pose;
    float distance = gaden::length(movementDir);
    movementDir = gaden::normalized(movementDir);

    // Traverse path
    int steps = ceil(distance / environment.description.cell_size); // Make sure no two iteration steps are separated more than 1 cell
    float increment = distance / steps;

    for (int i = 0; i < steps; i++)
    {
        // Determine point in space to evaluate
        gaden::Vector3 previous = filament.pose;
        filament.pose += movementDir * increment;

        // Check if the cell is occupied
        gaden::CellState cellState = check_pose_with_environment(filament.pose.x, filament.pose.y, filament.pose.z);
        if (cellState != gaden::CellState::Free)
        {
            filament.pose = previous;
            return cellState;
        }
    }

    // Direct line of sight confirmed!
    return gaden::CellState::Free;
}

// Update the filaments location in the 3D environment
//  According to Farrell Filament model, a filament is afected by three components of the wind flow.
//  1. Va (large scale wind) -> Advection (Va) -> Movement of a filament as a whole by wind) -> from CFD
//  2. Vm (middle scale wind)-> Movement of the filament with respect the center of the "plume" -> modeled as white noise
//  3. Vd (small scale wind) -> Difussion or change of the filament shape (growth with time)
//  We also consider Gravity and Bouyant Forces given the gas molecular mass
void CFilamentSimulator::update_filament_location(int i)
{
    // Estimte filament acceleration due to gravity & Bouyant force (for the given gas_type):
    constexpr float g = 9.8;
    constexpr float specific_gravity_air = 1; //[dimensionless]
    float accel = g * (specific_gravity_air - SpecificGravity[gasType]) / SpecificGravity[gasType];

    try
    {
        CFilament& filament = filaments[i];
        // Get 3D cell of the filament center
        int x_idx = floor((filament.pose.x - environment.description.min_coord.x) / environment.description.cell_size);
        int y_idx = floor((filament.pose.y - environment.description.min_coord.y) / environment.description.cell_size);
        int z_idx = floor((filament.pose.z - environment.description.min_coord.z) / environment.description.cell_size);

        // 1. Simulate Advection (Va)
        //    Large scale wind-eddies -> Movement of a filament as a whole by wind
        //------------------------------------------------------------------------
        const gaden::Vector3& windVec = wind[indexFrom3D(x_idx, y_idx, z_idx)];
        float newpos_x = filament.pose.x + windVec.x * time_step;
        float newpos_y = filament.pose.y + windVec.y * time_step;
        float newpos_z = filament.pose.z + windVec.z * time_step;

        // 2. Simulate Gravity & Bouyant Force
        //------------------------------------
        // OLD approach: using accelerations (pure gas)
        // newpos_z = filament.pose_z + 0.5*accel*pow(time_step,2);

        // Approximation from "Terminal Velocity of a Bubble Rise in a Liquid Column", World Academy of Science, Engineering and Technology 28 2007
        constexpr float ro_air = 1.205; //[kg/m³] density of air
        constexpr float mu = 19 * 1e-6; //[kg/s·m] dynamic viscosity of air
        float terminal_buoyancy_velocity = (g * (1 - SpecificGravity[gasType]) * ro_air * filament_ppm_center * pow(10, -6)) / (18 * mu);
        // newpos_z += terminal_buoyancy_velocity*time_step;

        // 3. Add some variability (stochastic process)
        //------------------------------------

        static thread_local std::mt19937 engine;
        static thread_local std::normal_distribution<> dist{0, filament_noise_std};

        newpos_x += dist(engine);
        newpos_y += dist(engine);
        newpos_z += dist(engine);

        // 4. Check filament location
        //------------------------------------
        gaden::CellState destinationState = moveFilament(filament, newpos_x, newpos_y, newpos_z);
        if (destinationState == gaden::CellState::Outlet)
        {
            // The location corresponds to an outlet! Delete filament!
            filament.valid = false;
        }

        // 4. Filament growth with time (this affects the posterior estimation of gas concentration at each cell)
        //    Vd (small scale wind eddies) -> Difussion or change of the filament shape (growth with time)
        //    R = sigma of a 3D gaussian -> Increasing sigma with time
        //------------------------------------------------------------------------
        filament.sigma = sqrt(filament_initial_std * filament_initial_std //
                              + filament_growth_gamma * (sim_time - filament.birth_time));
    }
    catch (std::exception& e)
    {
        GADEN_WARN("Exception Updating Filaments: {}", e.what());
        return;
    }
}

//==========================//
//                          //
//==========================//
void CFilamentSimulator::update_filaments_location()
{
#pragma omp parallel for
    for (int i = 0; i < current_number_filaments; i++)
    {
        if (filaments[i].valid)
        {
            update_filament_location(i);
        }
    }
    current_number_filaments += floor(numFilament_aux);
    numFilament_aux -= floor(numFilament_aux);
}

//==========================//
//                          //
//==========================//
void CFilamentSimulator::publish_markers()
{
    // 1. Clean old markers
    filament_marker.points.clear();
    filament_marker.colors.clear();
    filament_marker.header.stamp = now();
    filament_marker.pose.orientation.w = 1.0;

    // width of points: scale.x is point width, scale.y is point height
    filament_marker.scale.x = environment.description.cell_size / 4;
    filament_marker.scale.y = environment.description.cell_size / 4;
    filament_marker.scale.z = environment.description.cell_size / 4;

    // 2. Add a marker for each filament!
    for (int i = 0; i < current_number_filaments; i++)
    {
        geometry_msgs::msg::Point point;
        std_msgs::msg::ColorRGBA color;

        // Set filament pose
        point.x = filaments[i].pose.x;
        point.y = filaments[i].pose.y;
        point.z = filaments[i].pose.z;

        // Set filament color
        color.a = 1;
        if (filaments[i].valid)
        {
            color.r = 0;
            color.g = 0;
            color.b = 1;
        }
        else
        {
            color.r = 1;
            color.g = 0;
            color.b = 0;
        }

        // Add marker
        filament_marker.points.push_back(point);
        filament_marker.colors.push_back(color);
    }

    // Publish marker of the filaments
    marker_pub->publish(filament_marker);
}

//==========================//
//                          //
//==========================//
float CFilamentSimulator::random_number(float min_val, float max_val)
{
    float n = (float)(rand() % 100); // int random number [0, 100)
    n = n / 100.0f;                  // random number [0, 1)
    n = n * (max_val - min_val);     // random number [0, max-min)
    n = n + min_val;                 // random number [min, max)
    return n;
}

bool eq(float a, float b)
{
    return abs(a - b) < 0.001;
}

// Saves current Wind + GasConcentration to file
//  These files will be later used in the "player" node.
void CFilamentSimulator::save_state_to_file()
{
    last_saved_step++;
    last_saved_timestamp = sim_time;
    // Configure file name for saving the current snapshot
    std::string out_filename = fmt::format("{}/iteration_{}", results_location, last_saved_step);

    FILE* file = fopen(out_filename.c_str(), "wb");
    if (file == NULL)
    {
        GADEN_WARN("CANNOT OPEN LOG FILE\n");
        exit(1);
    }
    fclose(file);

    boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;
    std::stringstream ist;
    inbuf.push(boost::iostreams::zlib_compressor());
    inbuf.push(ist);

    ist.write((char*)&gaden::version_major, sizeof(int));
    ist.write((char*)&gaden::version_minor, sizeof(int));

    ist.write((char*)&environment.description, sizeof(environment.description));

    ist.write((char*)&gas_source_pos, sizeof(gas_source_pos));

    ist.write((char*)&gasType, sizeof(int));

    // constants to work out the gas concentration form the filament location
    ist.write((char*)&filament_numMoles_of_gas, sizeof(filament_numMoles_of_gas));
    float num_moles_all_gases_in_cm3 = env_cell_numMoles / env_cell_vol;
    ist.write((char*)&num_moles_all_gases_in_cm3, sizeof(num_moles_all_gases_in_cm3));

    ist.write((char*)&last_wind_idx, sizeof(int)); // index of the wind file (they are stored separately under (results_location)/wind/... )

    for (int i = 0; i < filaments.size(); i++)
    {
        if (filaments[i].valid)
        {
            ist.write((char*)&i, sizeof(int));
            ist.write((char*)&filaments[i].pose, sizeof(gaden::Vector3));
            ist.write((char*)&filaments[i].sigma, sizeof(float));
        }
    }

    std::ofstream fi(out_filename);
    boost::iostreams::copy(inbuf, fi);
    fi.close();
}

size_t CFilamentSimulator::indexFrom3D(int x, int y, int z)
{
    return gaden::indexFrom3D(gaden::Vector3i(x, y, z), environment.description.dimensions);
}

//==============================//
//			MAIN                //
//==============================//
int main(int argc, char** argv)
{
    // Init ROS-NODE
    rclcpp::init(argc, argv);

    gaden::Utils::Time::Stopwatch stopwatch;

    // Create simulator obj and initialize it
    std::shared_ptr<CFilamentSimulator> sim = std::make_shared<CFilamentSimulator>();

    // Initiate Random Number generator with current time
    srand(time(NULL));

    //--------------
    // LOOP
    //--------------
    while (rclcpp::ok() && (sim->current_simulation_step < sim->numSteps))
    {
        GADEN_TRACE("Simulating step {} (sim_time = {:.2f})", sim->current_simulation_step, sim->sim_time);

        // 0. Load wind snapshot (if necessary and availabe)
        if (sim->sim_time - sim->sim_time_last_wind >= sim->windTime_step)
        {
            // Time to update wind!
            sim->sim_time_last_wind = sim->sim_time;
            if (sim->allow_looping)
            {
                // Load wind-data
                sim->read_wind_snapshot(sim->current_wind_snapshot);
                // Update idx
                if (sim->current_wind_snapshot >= sim->loop_to_step)
                {
                    sim->current_wind_snapshot = sim->loop_from_step;
                    sim->wind_finished = true;
                }
                else
                    sim->current_wind_snapshot++;
            }
            else
                sim->read_wind_snapshot(floor(sim->sim_time / sim->windTime_step)); // Alllways increasing
        }

        // 1. Create new filaments close to the source location
        //    On each iteration num_filaments (See params) are created
        sim->add_new_filaments(sim->environment.description.cell_size);

        // 2. Publish markers for RVIZ
        sim->publish_markers();

        // 3. Update filament locations
        sim->update_filaments_location();

        // 4. Save data (if necessary)
        if ((sim->save_results == 1) && (sim->sim_time >= sim->results_min_time))
        {
            float time_next_save = sim->results_time_step + sim->last_saved_timestamp;
            if (sim->sim_time > time_next_save || std::abs(sim->sim_time - time_next_save) < 0.01)
                sim->save_state_to_file();
        }

        // 5. Update Simulation state
        sim->sim_time = sim->sim_time + sim->time_step; // sec
        sim->current_simulation_step++;

        rclcpp::spin_some(sim);
    }

    if (rclcpp::ok())
    {
        GADEN_INFO_COLOR(fmt::terminal_color::blue, "Filament simulator finished correctly! Ran for {:.2f}s", stopwatch.ellapsed());
    }
}
