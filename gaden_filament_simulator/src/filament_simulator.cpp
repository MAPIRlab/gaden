#define GADEN_LOGGER_ID "FilamentSimulator"
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
 ---------------------------------------------------------------------------------------*/

#include "filament_simulator.h"
#include "gaden/AirflowDisturbance.hpp"
#include "gaden/RunningSimulation.hpp"
#include "gaden/core/Logging.hpp"
#include "gaden/datatypes/sources/PointSource.hpp"
#include "gaden/internal/PathUtils.hpp"
#include "gaden/internal/Time.hpp"
#include <gaden_common/Visualization.hpp>

using namespace gaden;

//==============================//
//			MAIN                //
//==============================//
int main(int argc, char** argv)
{
    // Init ROS-NODE
    rclcpp::init(argc, argv);

    // Create simulator obj and initialize it
    auto sim = std::make_shared<FilamentSimulator>();

    gaden::Utils::Time::Stopwatch stopwatch;
    sim->Run();
    if (rclcpp::ok())
        GADEN_INFO_COLOR(fmt::terminal_color::blue, "Filament simulator finished correctly! Ran for {:.2f}s", stopwatch.ellapsed());
}

void FilamentSimulator::Run()
{
    rotorPositionSub = create_subscription<geometry_msgs::msg::PointStamped>("/gaden/rotorPosition", 1,
                                                                             std::bind(&FilamentSimulator::AddAirflowDisturbance, this, std::placeholders::_1));

    float maxSimTime = getParameter("sim_time", 300.f);

    RunningSimulation::Parameters params;
    std::shared_ptr<EnvironmentConfiguration> envConfig;

    std::filesystem::path projectPath = GadenUtils::getParam<std::string>(shared_from_this(), "projectPath", "");
    if (std::filesystem::exists(projectPath))
    {
        gadenProject.emplace(projectPath);
        GADEN_CHECK_RESULT(gadenProject->ReadDirectory());
        params = gadenProject->GetSimulationParams(getParameter<std::string>("simulationID", "sim"));
        envConfig = EnvironmentConfiguration::ReadDirectory(projectPath);
        if (!envConfig)
        {
            GADEN_ERROR("Could not read environment configuration from '{}'", projectPath);
            GADEN_TERMINATE;
        }
    }

    // if we don't have a gaden project directory (using old configurations) read the info from ros parameters
    if (!gadenProject)
    {
        GadenUtils::OldProjectWarning();

        params = {
            .source = std::make_shared<gaden::PointSource>(),
            .deltaTime = getParameter("time_step", 0.1f),
            .windIterationDeltaTime = getParameter("wind_time_step", 1.0f),
            .temperature = getParameter("wind_time_step", 298.0f),
            .pressure = getParameter("wind_time_step", 1.0f),
            .filamentPPMcenter_initial = getParameter("ppm_filament_center", 20.0f),
            .filamentInitialSigma = getParameter("filament_initial_std", 1.5f),
            .filamentGrowthGamma = getParameter("filament_growth_gamma", 10.0f),
            .filamentNoise_std = getParameter("filament_noise_std", 0.1f),
            .numFilaments_sec = static_cast<float>(getParameter("num_filaments_sec", 100)),
            .expectedNumIterations = static_cast<size_t>(std::ceil(maxSimTime / params.deltaTime)),
            .windLoop = LoopConfig{.loop = getParameter("allow_looping", false),                   //
                                   .from = static_cast<size_t>(getParameter("loop_from_step", 1)), //
                                   .to = static_cast<size_t>(getParameter("loop_to_step", 100))},
            .saveResults = static_cast<bool>(getParameter("save_results", 1)),
            .saveDeltaTime = getParameter("results_time_step", 0.5f),
            .saveDataDirectory = getParameter<std::string>("results_location", ""),
        };

        params.source->gasType = static_cast<GasType>(getParameter("gas_type", 0));
        params.source->sourcePosition = Vector3{
            getParameter("source_position_x", 0.0),
            getParameter("source_position_y", 0.0),
            getParameter("source_position_z", 0.0),
        };

        gaden::paths::TryCreateDirectory(params.saveDataDirectory);

        std::vector<std::filesystem::path> windFiles;
        std::filesystem::path environmentFile;
        std::string wind_data = getParameter<std::string>("wind_data", "");
        windFiles = GetWindFilePaths(wind_data);

        // some *really* old launch files (like VGR) followed this annoying convention where the parameter ended with an underscore
        if (windFiles.empty() && wind_data.back() == '_')
        {
            GADEN_WARN("Trying to find wind files with the naming convention of old projects (pre gaden 2.0)");
            std::string old_wind_data = wind_data;
            old_wind_data.erase(old_wind_data.size() - 1);
            windFiles = GetWindFilePaths(old_wind_data);
        }

        environmentFile = getParameter<std::string>("occupancy3D_data", "");

        envConfig = std::make_shared<gaden::EnvironmentConfiguration>();
        GADEN_CHECK_RESULT(envConfig->environment.ReadFromFile(environmentFile));
        envConfig->windSequence.Initialize(windFiles, envConfig->environment.numCells(), params.windLoop ? *params.windLoop : LoopConfig{});
    }

    sim.emplace(params, envConfig);
    sim->gasDisplayColor = {.r = 0, .g = 0, .b = 1, .a = 1}; // do we want to bother reading this as a parameter?

    // old launch files require the wind data to be copied inside the results folder
    if (!gadenProject)
        envConfig->windSequence.WriteToFiles(params.saveDataDirectory / "wind", "wind_iteration");

    // Start the simulation
    //--------------------------
    bool limitRate = getParameter("limitRate", false);
    float runRate = limitRate ? getParameter("runRate", 10.0) : 10.0;
    rclcpp::Rate rate(runRate);
    while (rclcpp::ok() && sim->GetCurrentTime() < maxSimTime)
    {
        sim->AdvanceTimestep();
        const auto& filaments = sim->GetFilaments();
        publishMarkers(filaments);

        rclcpp::spin_some(shared_from_this());
        if (limitRate)
            rate.sleep();
    }
    sim = std::nullopt;
}

void FilamentSimulator::AddAirflowDisturbance(const geometry_msgs::msg::PointStamped::SharedPtr rotorPosition)
{
    if (!sim)
        return;

    Vector3 dronePosition(rotorPosition->point.x, rotorPosition->point.y, rotorPosition->point.z);
    gaden::Airflow::QuadrotorDisturbanceFarField::ModifyField(
        dronePosition,
        sim->config->localAirflowDisturbances,
        sim->config->environment,
        0.12, 0.23, 0.07,
        sim->GetParameters().pressure,
        sim->GetParameters().temperature);
}

void FilamentSimulator::publishMarkers(std::vector<Filament> const& filaments)
{
    Marker filament_marker;
    if (!gasPublisher)
    {
        gasPublisher = create_publisher<Marker>("filament_visualization", 1);
        sourcePublisher = create_publisher<MarkerArray>("source_visualization", 1);
    }

    // 1. Clean old markers
    filament_marker.header.stamp = now();
    filament_marker.header.frame_id = "map";
    filament_marker.type = filament_marker.POINTS;

    // width of points: scale.x is point width, scale.y is point height
    filament_marker.scale.x = 0.02;
    filament_marker.scale.y = 0.02;
    filament_marker.scale.z = 0.02;

    // 2. Add a marker for each filament!
    for (int i = 0; i < filaments.size(); i++)
    {
        geometry_msgs::msg::Point point;
        std_msgs::msg::ColorRGBA color;

        // Set filament pose
        point.x = filaments[i].position.x;
        point.y = filaments[i].position.y;
        point.z = filaments[i].position.z;

        // Set filament color
        color = GadenUtils::toRosColor(sim->gasDisplayColor);

        // Add marker
        filament_marker.points.push_back(point);
        filament_marker.colors.push_back(color);
    }

    // Publish marker of the filaments
    gasPublisher->publish(filament_marker);

    Marker sourceMarker = GadenUtils::MarkerSourcePosition(this, *sim);
    sourcePublisher->publish(MarkerArray().set__markers({sourceMarker}));
}

// at different times, gaden has employed multiple rules for where to store / how to name wind files
// that means, if we want to still support old projects, we need to account for the different ways in which the path could be specified
// in theory, we could just require that old projects are re-configured with the new, much easier to use tools, but that would be annoying for users
std::vector<std::filesystem::path> FilamentSimulator::GetWindFilePaths(std::filesystem::path const& windFilesLocation)
{
    std::vector<std::filesystem::path> paths;
    GADEN_INFO_COLOR(fmt::terminal_color::blue, "Parameter 'wind_data': '{}'", windFilesLocation);

    // post 3.0
    GADEN_INFO("Trying to load wind files with pattern '{}/wind_iteration_i'", windFilesLocation);
    paths = GadenUtils::GetWindFiles([](std::string const& path, size_t idx)
                                     {
                                         return fmt::format("{}/wind_iteration_{}", path, idx);
                                     },
                                     windFilesLocation);
    if (!paths.empty())
    {
        GADEN_INFO("Found the wind files!");
        return paths;
    }

    // 2.6
    GADEN_INFO("Trying to load wind files with pattern '{}_i.csv_gaden'", windFilesLocation);
    paths = GadenUtils::GetWindFiles([](std::string const& path, size_t idx)
                                     {
                                         return fmt::format("{}_{}.csv_gaden", path, idx);
                                     },
                                     windFilesLocation);
    if (!paths.empty())
    {
        GADEN_INFO("Found the wind files!");
        return paths;
    }

    // Pre-2.6
    GADEN_INFO("Trying to load wind files with pattern '{}_i.csv_U'", windFilesLocation);
    paths = GadenUtils::GetWindFiles([](std::string const& path, size_t idx)
                                     {
                                         return fmt::format("{}_{}.csv_U", path, idx);
                                     },
                                     windFilesLocation);
    if (!paths.empty())
    {
        GADEN_INFO("Found the wind files!");
        return paths;
    }

    GADEN_ERROR("Could not find wind files!");
    return {};
}