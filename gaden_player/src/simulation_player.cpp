/*--------------------------------------------------------------------------------
 * Pkg for playing the simulation results of the "filament_simulator" pkg.
 * It allows to run on real time, and provide services to simulated sensors (gas, wind)
 * It supports loading several simulations at a time, which allows multiple gas sources and gas types
 * It also generates a point cloud representing the gas concentration [ppm] on the 3D environment
 --------------------------------------------------------------------------------*/
#define GADEN_LOGGER_ID "GadenPlayer"
#include <gaden_common/Utils.hpp>

#include "gaden/core/Assertions.hpp"
#include "gaden/internal/MathUtils.hpp"
#include "gaden/internal/PathUtils.hpp"
#include "gaden/internal/Time.hpp"
#include "simulation_player.h"

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

gaden_msgs::msg::GasInCell Player::GetAllGasesSingleCell(float x, float y, float z, const std::vector<gaden::GasType>& gas_types)
{
    std::map<gaden::GasType, float> concentrations = playbackScene->SampleConcentrations(gaden::Vector3(x, y, z));

    // Configure Response
    gaden_msgs::msg::GasInCell response;
    for (int i = 0; i < gas_types.size(); i++)
        response.concentration.push_back(concentrations.at(gas_types.at(i)));

    return response;
}

bool Player::GetGasValue_srv(gaden_msgs::srv::GasPosition::Request::SharedPtr req, gaden_msgs::srv::GasPosition::Response::SharedPtr res)
{
    std::vector<gaden::GasType> gasTypes = playbackScene->GetGasTypes();
    std::vector<std::string> gasNames;

    for (int i = 0; i < gasTypes.size(); i++)
        gasNames.push_back(gaden::to_string(gasTypes.at(i)));

    res->gas_type = gasNames;
    
    for (int i = 0; i < req->x.size(); i++)
        res->positions.push_back(GetAllGasesSingleCell(req->x[i], req->y[i], req->z[i], gasTypes));
    
    return true;
}

bool Player::GetWindValue_srv(gaden_msgs::srv::WindPosition::Request::SharedPtr req, gaden_msgs::srv::WindPosition::Response::SharedPtr res)
{
    // Since the wind fields are identical among different instances, return just the information from instance[0]
    for (int i = 0; i < req->x.size(); i++)
    {
        gaden::Vector3 windVec = playbackScene->SampleWind(gaden::Vector3{req->x[i], req->y[i], req->z[i]});
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
        GADEN_CHECK_RESULT(gadenProject->ReadDirectory());
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
            playbackScene->AdvanceTimestep();

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
    GadenUtils::OldProjectWarning();

    // Number of simulators to load (For simulating multiple gases and multiple sources)
    int num_simulators = declare_parameter<int>("num_simulators", 1);

    GADEN_INFO("num_simulators: {}", num_simulators);

    // FilePath for simulated data
    playbackMetadata.params.resize(num_simulators);
    GADEN_VERIFY(num_simulators >= 1, "Must have at least one simulation to play back!");
    int initial_iteration = declare_parameter<int>("initial_iteration", 1);

    for (int i = 0; i < num_simulators; i++)
    {
        // Get location of simulation data for instance (i)
        std::string paramName = fmt::format("simulation_data_{}", i);
        playbackMetadata.params[i].resultsDirectory = declare_parameter<std::string>(paramName.c_str(), "");
        playbackMetadata.params[i].startIteration = initial_iteration;
        GADEN_INFO("simulation_data_{}: {}", i, playbackMetadata.params[i].resultsDirectory);
    }

    playbackMetadata.gasDisplayColors.resize(num_simulators);
    for (int i = 0; i < num_simulators; i++)
    {
        // Get location of simulation data for instance (i)
        std::string paramName = fmt::format("gas_display_color_{}", i);
        auto colorAsVec = declare_parameter<std::vector<float>>(paramName.c_str(), {0, 1, 0});
        playbackMetadata.gasDisplayColors.at(i).r = colorAsVec.at(0);
        playbackMetadata.gasDisplayColors.at(i).g = colorAsVec.at(1);
        playbackMetadata.gasDisplayColors.at(i).b = colorAsVec.at(2);
        playbackMetadata.gasDisplayColors.at(i).a = 1;
    }

    // Initial iteration
    std::filesystem::path occupancyFile = declare_parameter<std::string>("occupancyFile", "");
    GADEN_CHECK_RESULT(environmentConfig.environment.ReadFromFile(occupancyFile));

    // Loop
    playbackMetadata.loop.loop = declare_parameter<bool>("allow_looping", false);
    playbackMetadata.loop.from = declare_parameter<int>("loop_from_iteration", 1);
    playbackMetadata.loop.to = declare_parameter<int>("loop_to_iteration", 1);
}

void Player::loadGadenProject()
{
    std::string playbackID = declare_parameter<std::string>("playbackID", "");
    try
    {
        playbackMetadata = gadenProject->playbacks.at(playbackID);
        playbackMetadata.gasDisplayColors.resize(playbackMetadata.gasDisplayColors.size());

        std::filesystem::path occupancyFile = gadenProject->rootDirectory / "OccupancyGrid3D.csv";
        GADEN_CHECK_RESULT(environmentConfig.environment.ReadFromFile(occupancyFile));
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
    std::filesystem::path pathOldProjects = playbackMetadata.params.at(0).resultsDirectory / "wind";                                           // in projects generated pre-3.0 the wind is in a subdirectory of gas_simulations
    std::filesystem::path pathNewProjects = playbackMetadata.params.at(0).resultsDirectory.parent_path().parent_path().parent_path() / "wind"; // in projects generated post-3.0 the wind is in a subdirectory of the environment configuration, above gas_simulations
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
    playbackScene.emplace(playbackMetadata, environmentConfig);
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

    const auto& simulations = playbackScene->GetSimulations();
    for (int i = 0; i < simulations.size(); i++)
    {
        auto const& filaments = simulations[i].GetFilaments();
        size_t count = FillMarkerArray(marker.points, filaments);
        for (size_t pointIdx = 0; pointIdx < count; pointIdx++)
            marker.colors.push_back(GadenUtils::toRosColor(playbackScene->GetColors().at(i)));
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
