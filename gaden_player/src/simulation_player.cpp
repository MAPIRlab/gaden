/*--------------------------------------------------------------------------------
 * Pkg for playing the simulation results of the "filament_simulator" pkg.
 * It allows to run on real time, and provide services to simulated sensors (gas, wind)
 * It supports loading several simulations at a time, which allows multiple gas sources and gas types
 * It also generates a point cloud representing the gas concentration [ppm] on the 3D environment
 --------------------------------------------------------------------------------*/
#define GADEN_LOGGER_ID "GadenPlayer"

#include "simulation_player.h"
#include "gaden/core/Assertions.hpp"
#include "gaden/internal/MathUtils.hpp"
#include "gaden/internal/PathUtils.hpp"
#include "gaden/internal/Time.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<Player> player = std::make_shared<Player>();

    player->run();

    return 0;
}

Player::Player()
    : rclcpp::Node("gaden_player")
{}

//--------------- SERVICES CALLBACKS----------------------//

gaden_msgs::msg::GasInCell Player::GetAllGasesSingleCell(float x, float y, float z, const std::vector<std::string>& gas_types)
{
    std::vector<double> srv_response_gas_concs(simulations.size());
    std::map<std::string, double> concentrationByGasType;
    for (int i = 0; i < gas_types.size(); i++)
        concentrationByGasType[gas_types[i]] = 0;

    // Get all gas concentrations and gas types (from all instances)
    for (int i = 0; i < simulations.size(); i++)
    {
        std::string gasName = gaden::to_string(simulations.at(i).simulationMetadata.gasType);
        concentrationByGasType[gasName] += simulations.at(i).SampleConcentration({x, y, z});
    }

    // Configure Response
    gaden_msgs::msg::GasInCell response;
    for (int i = 0; i < gas_types.size(); i++)
    {
        response.concentration.push_back(concentrationByGasType[gas_types[i]]);
    }
    return response;
}

bool Player::GetGasValue_srv(gaden_msgs::srv::GasPosition::Request::SharedPtr req, gaden_msgs::srv::GasPosition::Response::SharedPtr res)
{
    std::set<std::string> gas_types;

    for (int i = 0; i < simulations.size(); i++)
        gas_types.insert(gaden::to_string(simulations.at(i).simulationMetadata.gasType));

    std::vector<std::string> gast_types_v(gas_types.begin(), gas_types.end());
    res->gas_type = gast_types_v;
    for (int i = 0; i < req->x.size(); i++)
    {
        res->positions.push_back(GetAllGasesSingleCell(req->x[i], req->y[i], req->z[i], gast_types_v));
    }
    return true;
}

bool Player::GetWindValue_srv(gaden_msgs::srv::WindPosition::Request::SharedPtr req, gaden_msgs::srv::WindPosition::Response::SharedPtr res)
{
    // Since the wind fields are identical among different instances, return just the information from instance[0]
    for (int i = 0; i < req->x.size(); i++)
    {
        gaden::Vector3 windVec = simulations[0].SampleWind(gaden::Vector3{req->x[i], req->y[i], req->z[i]});
        res->u.push_back(windVec.x);
        res->v.push_back(windVec.y);
        res->w.push_back(windVec.z);
    }
    return true;
}

//------------------------ MAIN --------------------------//
void Player::run()
{
    // if there is a gaden project directory, we will just parse those files instead of reading everything from ROS params
    std::filesystem::path projectPath = declare_parameter<std::string>("projectPath", "");
    if (std::filesystem::exists(projectPath))
    {
        gadenProject.emplace(projectPath);
        GADEN_CHECK_RESULT(gadenProject->Read());
    }

    // Read Node Parameters
    if (gadenProject)
        loadGadenProject();
    else
        loadNodeParameters();

    // Init variables
    initSimulations();
    rclcpp::Time time_last_loaded_file = now();
    srand(time(NULL)); // initialize random seed

    // Services offered
    auto serviceGas = create_service<gaden_msgs::srv::GasPosition>(
        "odor_value", std::bind(&Player::GetGasValue_srv, this, std::placeholders::_1, std::placeholders::_2));
    auto serviceWind = create_service<gaden_msgs::srv::WindPosition>(
        "wind_value", std::bind(&Player::GetWindValue_srv, this, std::placeholders::_1, std::placeholders::_2));

    // Loop
    rclcpp::Rate r(100); // Set max rate at 100Hz (for handling services - Top Speed!!)
    auto shared_this = shared_from_this();

    // playback frequency
    float player_freq = declare_parameter<float>("player_freq", 1); // Hz
    GADEN_INFO("player_freq {:.2f} Hz", player_freq);
    gaden::Utils::Time::Countdown countdown(1.f / player_freq);

    while (rclcpp::ok())
    {
        if (countdown.isDone())
        {
            // Read Gas and Wind data from log_files
            for (auto& sim : simulations)
                sim.AdvanceTimestep();

            displayCurrentGasDistribution(); // Rviz visualization

            // Looping?
            time_last_loaded_file = now();
            countdown.Restart();
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
    // Number of simulators to load (For simulating multiple gases and multiple sources)
    int num_simulators = declare_parameter<int>("num_simulators", 1);

    GADEN_INFO("num_simulators: {}", num_simulators);

    // FilePath for simulated data
    params.resize(num_simulators);
    GADEN_VERIFY(num_simulators >= 1, "Must have at least one simulation to play back!");
    int initial_iteration = declare_parameter<int>("initial_iteration", 1);

    for (int i = 0; i < num_simulators; i++)
    {
        // Get location of simulation data for instance (i)
        std::string paramName = fmt::format("simulation_data_{}", i);
        params[i].resultsDirectory = declare_parameter<std::string>(paramName.c_str(), "");
        params[i].startIteration = initial_iteration;
        GADEN_INFO("simulation_data_{}: {}", i, params[i].resultsDirectory);
    }

    gasDisplayColors.resize(num_simulators);
    for (int i = 0; i < num_simulators; i++)
    {
        // Get location of simulation data for instance (i)
        std::string paramName = fmt::format("gas_display_color_{}", i);
        auto colorAsVec = declare_parameter<std::vector<float>>(paramName.c_str(), {0, 1, 0});
        gasDisplayColors.at(i).r = colorAsVec.at(0);
        gasDisplayColors.at(i).g = colorAsVec.at(1);
        gasDisplayColors.at(i).b = colorAsVec.at(2);
        gasDisplayColors.at(i).a = 1;
    }

    // Initial iteration
    std::filesystem::path occupancyFile = declare_parameter<std::string>("occupancyFile", "");
    GADEN_CHECK_RESULT(environmentConfig.environment.ReadFromFile(occupancyFile));

    // Loop
    loopConfig.loop = declare_parameter<bool>("allow_looping", false);
    loopConfig.from = declare_parameter<int>("loop_from_iteration", 1);
    loopConfig.to = declare_parameter<int>("loop_to_iteration", 1);
}

void Player::loadGadenProject()
{
    std::string playbackID = declare_parameter<std::string>("playbackID", "");
    try
    {
        gaden::Project::PlaybackMetadata metadata = gadenProject->playbacks.at(playbackID);
        params = metadata.params;
        gasDisplayColors.resize(metadata.gasDisplayColor.size());
        for (size_t i = 0; i < gasDisplayColors.size(); i++)
        {
            gasDisplayColors[i].r = metadata.gasDisplayColor[i].r;
            gasDisplayColors[i].g = metadata.gasDisplayColor[i].g;
            gasDisplayColors[i].b = metadata.gasDisplayColor[i].b;
            gasDisplayColors[i].a = metadata.gasDisplayColor[i].a;
        }
        std::filesystem::path occupancyFile = gadenProject->rootDirectory / "OccupancyGrid3D.csv";
        GADEN_CHECK_RESULT(environmentConfig.environment.ReadFromFile(occupancyFile));
        loopConfig = metadata.loop;
    }
    catch (std::exception const& e)
    {
        GADEN_ERROR("Could not find a playback configuration with the name '{}'", playbackID);
        GADEN_TERMINATE;
    }
}

void Player::initSimulations()
{
    // Find the wind files
    // We only need to find them once, since the wind is assumed to be identical for all simulation instances
    std::filesystem::path pathOldProjects = params.at(0).resultsDirectory / "wind";                                           // in projects generated pre-3.0 the wind is in a subdirectory of gas_simulations
    std::filesystem::path pathNewProjects = params.at(0).resultsDirectory.parent_path().parent_path().parent_path() / "wind"; // in projects generated post-3.0 the wind is in a subdirectory of the environment configuration, above gas_simulations
    std::vector<std::filesystem::path> windFiles;
    GADEN_INFO("Looking for wind files in:\n"
               "\t-'{}'\n"
               "\t-'{}'",
               pathNewProjects, pathOldProjects);

    if (std::filesystem::exists(pathNewProjects))
    {
        GADEN_INFO("Found wind files at '{}'", pathNewProjects);
        windFiles = gaden::paths::GetAllFilesInDirectory(pathNewProjects);
    }
    else if (std::filesystem::exists(pathOldProjects))
    {
        GADEN_INFO("Found wind files at '{}'", pathOldProjects);
        windFiles = gaden::paths::GetAllFilesInDirectory(pathOldProjects);
    }
    else
    {
        GADEN_ERROR("Could not find wind files!");
        GADEN_TERMINATE;
    }

    environmentConfig.windSequence.Initialize(windFiles, environmentConfig.environment.numCells(), {});

    for (size_t i = 0; i < params.size(); i++)
    {
        simulations.emplace_back(params.at(i), environmentConfig, loopConfig);
    }
}

// Display in RVIZ the gas distribution
void Player::displayCurrentGasDistribution()
{
    static auto markerPub = create_publisher<visualization_msgs::msg::Marker>("Gas_Distribution", 1);
    static visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.ns = "Gas_Dispersion";
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::POINTS; // Marker type
    marker.id = 0;                                         // One marker with multiple points.
    marker.scale.x = 0.025;
    marker.scale.y = 0.025;
    marker.scale.z = 0.025;

    // Remove previous data points
    marker.points.clear();
    marker.colors.clear();
    for (int i = 0; i < simulations.size(); i++)
    {
        auto const& filaments = simulations[i].GetFilaments();
        size_t count = FillMarkerArray(marker.points, filaments);
        for (size_t pointIdx = 0; pointIdx < count; pointIdx++)
            marker.colors.push_back(gasDisplayColors.at(i));
    }

    // Display particles
    markerPub->publish(marker);
}

//==================================== SIM_OBJ ==============================//

size_t Player::FillMarkerArray(std::vector<geometry_msgs::msg::Point>& points, std::vector<gaden::Filament> const& filaments)
{
    size_t count = 0;
    for (auto it = filaments.begin(); it != filaments.end(); it++)
    {
        geometry_msgs::msg::Point p; // Location of point

        const gaden::Filament& filament = *it;
        for (int i = 0; i < 5; i++)
        {
            p.x = (filament.position.x) + gaden::uniformRandom(-filament.sigma / 50, filament.sigma / 50);
            p.y = (filament.position.y) + gaden::uniformRandom(-filament.sigma / 50, filament.sigma / 50);
            p.z = (filament.position.z) + gaden::uniformRandom(-filament.sigma / 50, filament.sigma / 50);

            // Add particle marker
            points.push_back(p);
            count++;
        }
    }
    return count;
}
