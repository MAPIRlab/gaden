/*--------------------------------------------------------------------------------
 * Pkg for playing the simulation results of the "filament_simulator" pkg.
 * It allows to run on real time, and provide services to simulated sensors (gas, wind)
 * It supports loading several simulations at a time, which allows multiple gas sources and gas types
 * It also generates a point cloud representing the gas concentration [ppm] on the 3D environment
 --------------------------------------------------------------------------------*/

#include <boost/format.hpp>
#include "simulation_player.h"
#include <filesystem>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#define GADEN_LOGGER_ID "GadenPlayer"
#include <gaden_common/Logging.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<Player> player = std::make_shared<Player>();

    player->run();

    return 0;
}

Player::Player() : rclcpp::Node("gaden_player")
{
}

//--------------- SERVICES CALLBACKS----------------------//

gaden_player::msg::GasInCell Player::get_all_gases_single_cell(float x, float y, float z, const std::vector<std::string>& gas_types)
{
    std::vector<double> srv_response_gas_concs(num_simulators);
    std::map<std::string, double> concentrationByGasType;
    for (int i = 0; i < gas_types.size(); i++)
        concentrationByGasType[gas_types[i]] = 0;

    // Get all gas concentrations and gas types (from all instances)
    for (int i = 0; i < num_simulators; i++)
        concentrationByGasType[player_instances[i].gas_type] += player_instances[i].get_gas_concentration(x, y, z);

    // Configure Response
    gaden_player::msg::GasInCell response;
    for (int i = 0; i < gas_types.size(); i++)
    {
        response.concentration.push_back(concentrationByGasType[gas_types[i]]);
    }
    return response;
}

bool Player::get_gas_value_srv(gaden_player::srv::GasPosition::Request::SharedPtr req, gaden_player::srv::GasPosition::Response::SharedPtr res)
{
    std::set<std::string> gas_types;

    for (int i = 0; i < num_simulators; i++)
        gas_types.insert(player_instances[i].gas_type);

    std::vector<std::string> gast_types_v(gas_types.begin(), gas_types.end());
    res->gas_type = gast_types_v;
    for (int i = 0; i < req->x.size(); i++)
    {
        res->positions.push_back(get_all_gases_single_cell(req->x[i], req->y[i], req->z[i], gast_types_v));
    }
    return true;
}

bool Player::get_wind_value_srv(gaden_player::srv::WindPosition::Request::SharedPtr req, gaden_player::srv::WindPosition::Response::SharedPtr res)
{
    // Since the wind fields are identical among different instances, return just the information from instance[0]
    for (int i = 0; i < req->x.size(); i++)
    {
        double u, v, w;
        player_instances[0].get_wind_value(req->x[i], req->y[i], req->z[i], u, v, w);
        res->u.push_back(u);
        res->v.push_back(v);
        res->w.push_back(w);
    }
    return true;
}

//------------------------ MAIN --------------------------//
void Player::run()
{
    // Read Node Parameters
    loadNodeParameters();

    // Init variables
    init_all_simulation_instances();
    rclcpp::Time time_last_loaded_file = now();
    srand(time(NULL)); // initialize random seed

    load_all_data_from_logfiles(0); // On the first time, we configure gas type, source pos, etc.

    // Init Markers for RVIZ visualization
    mkr_gas_points.header.frame_id = "map";
    mkr_gas_points.header.stamp = now();
    mkr_gas_points.ns = "Gas_Dispersion";
    mkr_gas_points.action = visualization_msgs::msg::Marker::ADD;
    mkr_gas_points.type = visualization_msgs::msg::Marker::POINTS; // Marker type
    mkr_gas_points.id = 0;                                         // One marker with multiple points.
    mkr_gas_points.scale.x = 0.025;
    mkr_gas_points.scale.y = 0.025;
    mkr_gas_points.scale.z = 0.025;
    mkr_gas_points.pose.orientation.w = 1.0;

    // Services offered
    auto serviceGas = create_service<gaden_player::srv::GasPosition>(
        "odor_value", std::bind(&Player::get_gas_value_srv, this, std::placeholders::_1, std::placeholders::_2));
    auto serviceWind = create_service<gaden_player::srv::WindPosition>(
        "wind_value", std::bind(&Player::get_wind_value_srv, this, std::placeholders::_1, std::placeholders::_2));

    // Publishers
    marker_pub = create_publisher<visualization_msgs::msg::Marker>("Gas_Distribution", 1);

    // Loop
    rclcpp::Rate r(100); // Set max rate at 100Hz (for handling services - Top Speed!!)
    int iteration_counter = initial_iteration;
    auto shared_this = shared_from_this();
    while (rclcpp::ok())
    {
        if ((now() - time_last_loaded_file).seconds() >= 1 / player_freq)
        {
            if (verbose)
                GADEN_INFO("Playing simulation iteration {}", iteration_counter);
            // Read Gas and Wind data from log_files
            load_all_data_from_logfiles(iteration_counter);
            display_current_gas_distribution(); // Rviz visualization
            iteration_counter++;

            // Looping?
            if (allow_looping)
            {
                if (iteration_counter >= loop_to_iteration)
                {
                    iteration_counter = loop_from_iteration;
                    if (verbose)
                        GADEN_INFO("Looping");
                }
            }
            time_last_loaded_file = now();
        }

        // Attend service request at max rate!
        // This allows sensors to have higher sampling rates than the simulation update
        rclcpp::spin_some(shared_this);
        r.sleep();
    }
}

// Load Node parameters
void Player::loadNodeParameters()
{
    // player_freq
    verbose = declare_parameter<bool>("verbose", false);

    // player_freq
    player_freq = declare_parameter<double>("player_freq", 1); // Hz

    // Number of simulators to load (For simulating multiple gases and multiple sources)
    num_simulators = declare_parameter<int>("num_simulators", 1);

    if (verbose)
    {
        GADEN_INFO("player_freq {:.2f}", player_freq);
        GADEN_INFO("num_simulators:  {}", num_simulators);
    }

    // FilePath for simulated data
    simulation_data.resize(num_simulators);
    for (int i = 0; i < num_simulators; i++)
    {
        // Get location of simulation data for instance (i)
        std::string paramName = fmt::format("simulation_data_{}", i);
        simulation_data[i] = declare_parameter<std::string>(paramName.c_str(), "");
        if (verbose)
            GADEN_INFO("simulation_data_{}:  {}", i, simulation_data[i].c_str());
    }

    // Initial iteration
    initial_iteration = declare_parameter<int>("initial_iteration", 1);
    occupancyFile = declare_parameter<std::string>("occupancyFile", "");

    // Loop
    allow_looping = declare_parameter<bool>("allow_looping", false);
    loop_from_iteration = declare_parameter<int>("loop_from_iteration", 1);
    loop_to_iteration = declare_parameter<int>("loop_to_iteration", 1);
}

// Init
void Player::init_all_simulation_instances()
{
    GADEN_INFO("Initializing {} instances", num_simulators);

    // At least one instance is needed which loads the wind field data!
    sim_obj so(simulation_data[0], true, shared_from_this(), occupancyFile);
    player_instances.push_back(so);

    // Create other instances, but do not save wind information! It is the same for all instances
    for (int i = 1; i < num_simulators; i++)
    {
        sim_obj so(simulation_data[i], false, shared_from_this(), occupancyFile);
        player_instances.push_back(so);
    }

    // Set size for service responses
}

// Load new Iteration of the Gas&Wind State on the 3d environment
void Player::load_all_data_from_logfiles(int sim_iteration)
{
    // Load corresponding data for each instance (i.e for every gas source)
    for (int i = 0; i < num_simulators; i++)
    {
        if (verbose)
            GADEN_INFO("Loading new data to instance {} (iteration {})", i, sim_iteration);
        player_instances[i].load_data_from_logfile(sim_iteration);
    }
}

// Display in RVIZ the gas distribution
void Player::display_current_gas_distribution()
{
    // Remove previous data points
    mkr_gas_points.points.clear();
    mkr_gas_points.colors.clear();
    for (int i = 0; i < num_simulators; i++)
    {
        player_instances[i].get_concentration_as_markers(mkr_gas_points);
    }
    // Display particles
    marker_pub->publish(mkr_gas_points);
}

//==================================== SIM_OBJ ==============================//

// Constructor
sim_obj::sim_obj(std::string filepath, bool load_wind_info, rclcpp::Node::SharedPtr node, std::string occupancy_filePath)
    : occupancyFile(occupancy_filePath)
{
    gas_type = "unknown";
    simulation_filename = filepath;
    source_pos_x = source_pos_y = source_pos_z = 0.0; // m
    load_wind_data = load_wind_info;
    first_reading = true;

    createHeatmapImage = node->declare_parameter<bool>("createHeatmapImage", false);
    heatmapPath = node->declare_parameter<std::string>("heatmapPath", "");
    heatmapHeight = node->declare_parameter<float>("heatmapHeight", 0);
    heatmapThreshold = node->declare_parameter<double>("heatmapThreshold", 0.1);
    heatMapIterations = node->declare_parameter<int>("heatMapIterations", 100);

    if (!std::filesystem::exists(simulation_filename))
    {
        GADEN_ERROR("Simulation folder does not exist: {}", simulation_filename.c_str());
        exit(-1);
    }
}

sim_obj::~sim_obj()
{
}

// Load a new file with Gas+Wind data
void sim_obj::load_data_from_logfile(int sim_iteration)
{
    std::string filename = fmt::format("{}/iteration_{}", simulation_filename, sim_iteration);
    FILE* fileCheck;
    if ((fileCheck = fopen(filename.c_str(), "rb")) == NULL)
    {
        GADEN_ERROR("File {} does not exist\n", filename.c_str());
        return;
    }
    fclose(fileCheck);

    std::ifstream infile(filename, std::ios_base::binary);
    boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;
    inbuf.push(boost::iostreams::zlib_decompressor());
    inbuf.push(infile);

    std::stringstream decompressed;
    boost::iostreams::copy(inbuf, decompressed);

    // check the version of gaden used to generate the file
    decompressed.read((char*)&environment.versionMajor, sizeof(int));
    if (environment.versionMajor == 1)
    {
        environment.versionMinor = 0;
        load_logfile_version_1(decompressed);
    }
    else
    {
        decompressed.read((char*)&environment.versionMinor, sizeof(int));
        load_logfile_version_2(decompressed);
    }

    infile.close();

    static int iterationCounter = 0;
    if (createHeatmapImage)
    {
        if (iterationCounter < heatMapIterations)
            updateHeatmap();
        else if (iterationCounter == heatMapIterations)
            writeHeatmapImage();

        iterationCounter++;
    }
}

void sim_obj::load_logfile_version_1(std::stringstream& decompressed)
{
    if (first_reading)
    {
        GADEN_WARN(
            "You are reading a log file that was generated with an old version of gaden. While it should work correctly, if you experience any bugs, "
            "this is likely the cause. You can just re-run the filament simulator node to generate an updated version of the simulation");
        // coordinates were initially written as doubles, but we want to read them as floats now, so we need a buffer
        double bufferDoubles[5];
        decompressed.read((char*)&bufferDoubles, 3 * sizeof(double));
        environment.description.min_coord.x = bufferDoubles[0];
        environment.description.min_coord.y = bufferDoubles[1];
        environment.description.min_coord.z = bufferDoubles[2];

        decompressed.read((char*)&bufferDoubles, 3 * sizeof(double));
        environment.description.max_coord.x = bufferDoubles[0];
        environment.description.max_coord.y = bufferDoubles[1];
        environment.description.max_coord.z = bufferDoubles[2];

        decompressed.read((char*)&environment.description.num_cells.x, sizeof(int));
        decompressed.read((char*)&environment.description.num_cells.y, sizeof(int));
        decompressed.read((char*)&environment.description.num_cells.z, sizeof(int));

        decompressed.read((char*)&bufferDoubles, 3 * sizeof(double));
        environment.description.cell_size = bufferDoubles[0];

        decompressed.read((char*)&bufferDoubles, 3 * sizeof(double)); // ground truth source position, we can ignore it

        int gas_type_index;
        decompressed.read((char*)&gas_type_index, sizeof(int));
        gas_type = gasTypesByCode[gas_type_index];

        decompressed.read((char*)&total_moles_in_filament, sizeof(double));
        decompressed.read((char*)&num_moles_all_gases_in_cm3, sizeof(double));

        configure_environment();
        first_reading = false;
    }
    else
    {
        // skip headers
        decompressed.seekg(14 * sizeof(double) + 5 * sizeof(int));
    }

    int wind_index;
    decompressed.read((char*)&wind_index, sizeof(int));

    activeFilaments.clear();
    int filament_index;
    double x, y, z, stdDev;
    while (decompressed.peek() != EOF)
    {

        decompressed.read((char*)&filament_index, sizeof(int));
        decompressed.read((char*)&x, sizeof(double));
        decompressed.read((char*)&y, sizeof(double));
        decompressed.read((char*)&z, sizeof(double));
        decompressed.read((char*)&stdDev, sizeof(double));

        std::pair<int, Filament> pair(filament_index, Filament(x, y, z, stdDev));
        activeFilaments.insert(pair);
    }

    load_wind_file(wind_index);
}

void sim_obj::load_logfile_version_2(std::stringstream& decompressed)
{
    if (first_reading)
    {
        decompressed.read((char*)&environment.description, sizeof(environment.description));
        Gaden::Vector3 source_position;
        decompressed.read((char*)&source_position, sizeof(source_position));

        int gas_type_index;
        decompressed.read((char*)&gas_type_index, sizeof(int));
        gas_type = gasTypesByCode[gas_type_index];

        decompressed.read((char*)&total_moles_in_filament, sizeof(double));
        decompressed.read((char*)&num_moles_all_gases_in_cm3, sizeof(double));
        configure_environment();
        first_reading = false;
    }
    else
    {
        // skip header
        decompressed.seekg(2 * sizeof(int)                   // version
                           + sizeof(environment.description) // description
                           + sizeof(Gaden::Vector3)          // source position
                           + sizeof(int)                     // gas type
                           + 2 * sizeof(double)              // moles constants
        );
    }

    int wind_index;
    decompressed.read((char*)&wind_index, sizeof(int));

    activeFilaments.clear();
    int filament_index;
    double x, y, z, stdDev;
    while (decompressed.peek() != EOF)
    {

        decompressed.read((char*)&filament_index, sizeof(int));
        decompressed.read((char*)&x, sizeof(double));
        decompressed.read((char*)&y, sizeof(double));
        decompressed.read((char*)&z, sizeof(double));
        decompressed.read((char*)&stdDev, sizeof(double));

        std::pair<int, Filament> pair(filament_index, Filament(x, y, z, stdDev));
        activeFilaments.insert(pair);
    }

    load_wind_file(wind_index);
}

void sim_obj::load_wind_file(int wind_index)
{
    if (wind_index == last_wind_idx)
        return;
    last_wind_idx = wind_index;

    std::ifstream infile(fmt::format("{}/wind/wind_iteration_{}", simulation_filename, wind_index), std::ios_base::binary);
    infile.read((char*)U.data(), sizeof(double) * U.size());
    infile.read((char*)V.data(), sizeof(double) * U.size());
    infile.read((char*)W.data(), sizeof(double) * U.size());
    infile.close();
}

// Get Gas concentration at lcoation (x,y,z)
double sim_obj::get_gas_concentration(float x, float y, float z)
{

    int xx, yy, zz;
    xx = (int)ceil((x - environment.description.min_coord.x) / environment.description.cell_size);
    yy = (int)ceil((y - environment.description.min_coord.y) / environment.description.cell_size);
    zz = (int)ceil((z - environment.description.min_coord.z) / environment.description.cell_size);

    if (xx < 0 || xx > environment.description.num_cells.x || yy < 0 || yy > environment.description.num_cells.y || zz < 0 ||
        zz > environment.description.num_cells.z)
    {
        GADEN_ERROR(
                     "Requested gas concentration at a point outside the environment ({}, {}, {}). Are you using the correct coordinates?\n", x, y,
                     z);
        return 0;
    }
    double gas_conc = 0;
    for (auto it = activeFilaments.begin(); it != activeFilaments.end(); it++)
    {
        Filament fil = it->second;
        double distSQR = (x - fil.x) * (x - fil.x) + (y - fil.y) * (y - fil.y) + (z - fil.z) * (z - fil.z);

        double limitDistance = fil.sigma * 5 / 100;
        if (distSQR < limitDistance * limitDistance && check_environment_for_obstacle(x, y, z, fil.x, fil.y, fil.z))
        {
            gas_conc += concentration_from_filament(x, y, z, fil);
        }
    }

    return gas_conc;
}

double sim_obj::concentration_from_filament(float x, float y, float z, Filament filament)
{
    // calculate how much gas concentration does one filament contribute to the queried location
    double sigma = filament.sigma;
    double distance_cm = 100 * sqrt(pow(x - filament.x, 2) + pow(y - filament.y, 2) + pow(z - filament.z, 2));

    double num_moles_target_cm3 =
        (total_moles_in_filament / (sqrt(8 * pow(M_PI, 3)) * pow(sigma, 3))) * exp(-pow(distance_cm, 2) / (2 * pow(sigma, 2)));

    double ppm = num_moles_target_cm3 / num_moles_all_gases_in_cm3 * 1000000; // parts of target gas per million

    return ppm;
}

bool sim_obj::check_environment_for_obstacle(double start_x, double start_y, double start_z, double end_x, double end_y, double end_z)
{
    // Check whether one of the points is outside the valid environment or is not free
    if (check_pose_with_environment(start_x, start_y, start_z) != 0)
    {
        return false;
    }
    if (check_pose_with_environment(end_x, end_y, end_z) != 0)
    {
        return false;
    }

    // Calculate normal displacement vector
    double vector_x = end_x - start_x;
    double vector_y = end_y - start_y;
    double vector_z = end_z - start_z;
    double distance = sqrt(vector_x * vector_x + vector_y * vector_y + vector_z * vector_z);
    vector_x = vector_x / distance;
    vector_y = vector_y / distance;
    vector_z = vector_z / distance;

    // Traverse path
    int steps = ceil(distance / environment.description.cell_size); // Make sure no two iteration steps are separated more than 1 cell
    double increment = distance / steps;

    for (int i = 1; i < steps - 1; i++)
    {
        // Determine point in space to evaluate
        double pose_x = start_x + vector_x * increment * i;
        double pose_y = start_y + vector_y * increment * i;
        double pose_z = start_z + vector_z * increment * i;

        // Determine cell to evaluate (some cells might get evaluated twice due to the current code
        int x_idx = floor((pose_x - environment.description.min_coord.x) / environment.description.cell_size);
        int y_idx = floor((pose_y - environment.description.min_coord.y) / environment.description.cell_size);
        int z_idx = floor((pose_z - environment.description.min_coord.z) / environment.description.cell_size);

        // Check if the cell is occupied
        if (environment.Env[indexFrom3D(x_idx, y_idx, z_idx)] != 0)
        {
            return false;
        }
    }

    // Direct line of sight confirmed!
    return true;
}

int sim_obj::check_pose_with_environment(double pose_x, double pose_y, double pose_z)
{
    // 1.1 Check that pose is within the boundingbox environment
    if (pose_x < environment.description.min_coord.x || pose_x > environment.description.max_coord.x ||
        pose_y < environment.description.min_coord.y || pose_y > environment.description.max_coord.y ||
        pose_z < environment.description.min_coord.z || pose_z > environment.description.max_coord.z)
        return 1;

    // Get 3D cell of the point
    int x_idx = (pose_x - environment.description.min_coord.x) / environment.description.cell_size;
    int y_idx = (pose_y - environment.description.min_coord.y) / environment.description.cell_size;
    int z_idx = (pose_z - environment.description.min_coord.z) / environment.description.cell_size;

    if (x_idx >= environment.description.num_cells.x || y_idx >= environment.description.num_cells.y || z_idx >= environment.description.num_cells.z)
        return 1;

    // 1.2. Return cell occupancy (0=free, 1=obstacle, 2=outlet)
    return environment.Env[indexFrom3D(x_idx, y_idx, z_idx)];
}

// Get Wind concentration at lcoation (x,y,z)
void sim_obj::get_wind_value(float x, float y, float z, double& u, double& v, double& w)
{
    if (load_wind_data)
    {
        int xx, yy, zz;
        xx = (int)ceil((x - environment.description.min_coord.x) / environment.description.cell_size);
        yy = (int)ceil((y - environment.description.min_coord.y) / environment.description.cell_size);
        zz = (int)ceil((z - environment.description.min_coord.z) / environment.description.cell_size);

        if (xx < 0 || xx > environment.description.num_cells.x || yy < 0 || yy > environment.description.num_cells.y || zz < 0 ||
            zz > environment.description.num_cells.z)
        {
            GADEN_ERROR("Requested gas concentration at a point outside the environment. Are you using the correct coordinates?\n");
            return;
        }

        // Set wind vectors from that cell
        u = U[indexFrom3D(xx, yy, zz)];
        v = V[indexFrom3D(xx, yy, zz)];
        w = W[indexFrom3D(xx, yy, zz)];
    }
    else
    {
        GADEN_WARN("Request to provide Wind information when No Wind data is available!!");
    }
}

// Init instances (for running multiple simulations)
void sim_obj::configure_environment()
{
    // Resize Gas Concentration container
    C.resize(environment.description.num_cells.x * environment.description.num_cells.y * environment.description.num_cells.z);
    heatmap.resize(environment.description.num_cells.x, std::vector<double>(environment.description.num_cells.y, 0));

    // Resize Wind info container (if necessary)
    if (load_wind_data)
    {

        U.resize(environment.description.num_cells.x * environment.description.num_cells.y * environment.description.num_cells.z);
        V.resize(environment.description.num_cells.x * environment.description.num_cells.y * environment.description.num_cells.z);
        W.resize(environment.description.num_cells.x * environment.description.num_cells.y * environment.description.num_cells.z);
    }

    Gaden::ReadResult result = Gaden::readEnvFile(occupancyFile, environment);
    if (result == Gaden::ReadResult::NO_FILE)
    {
        GADEN_ERROR("No occupancy file provided to Gaden-player node!");
        exit(-1);
    }
    else if (result == Gaden::ReadResult::READING_FAILED)
    {
        GADEN_ERROR("Something went wrong while parsing the file!");
        exit(-1);
    }
}

void sim_obj::get_concentration_as_markers(visualization_msgs::msg::Marker& mkr_points)
{
    for (auto it = activeFilaments.begin(); it != activeFilaments.end(); it++)
    {
        geometry_msgs::msg::Point p;    // Location of point
        std_msgs::msg::ColorRGBA color; // Color of point

        Filament filament = it->second;
        for (int i = 0; i < 5; i++)
        {
            p.x = (filament.x) + ((std::rand() % 1000) / 1000.0 - 0.5) * filament.sigma / 200;
            p.y = (filament.y) + ((std::rand() % 1000) / 1000.0 - 0.5) * filament.sigma / 200;
            p.z = (filament.z) + ((std::rand() % 1000) / 1000.0 - 0.5) * filament.sigma / 200;

            color.a = 1;
            color.r = 0;
            color.g = 1;
            color.b = 0;
            // Add particle marker
            mkr_points.points.push_back(p);
            mkr_points.colors.push_back(color);
        }
    }
}

int sim_obj::indexFrom3D(int x, int y, int z)
{
    return x + y * environment.description.num_cells.x + z * environment.description.num_cells.x * environment.description.num_cells.y;
}

void sim_obj::updateHeatmap()
{
#pragma omp parallel for collapse(2)
    for (int i = 0; i < heatmap.size(); i++)
    {
        for (int j = 0; j < heatmap[0].size(); j++)
        {
            double p_x = environment.description.min_coord.x + (i + 0.5) * environment.description.cell_size;
            double p_y = environment.description.min_coord.y + (j + 0.5) * environment.description.cell_size;
            double g_c = get_gas_concentration(p_x, p_y, heatmapHeight);
            if (g_c > heatmapThreshold)
                heatmap[i][j]++;
        }
    }
}

void sim_obj::writeHeatmapImage()
{
    cv::Mat image(cv::Size(heatmap.size(), heatmap[0].size()), CV_32F, cv::Scalar(0));

#pragma omp parallel for collapse(2)
    for (int i = 0; i < heatmap.size(); i++)
    {
        for (int j = 0; j < heatmap[0].size(); j++)
        {
            image.at<float>(heatmap[0].size() - 1 - j, i) = (heatmap[i][j] / heatMapIterations) * 255;
        }
    }

    cv::imwrite(heatmapPath, image);

    GADEN_WARN("Heatmap image saved");
}