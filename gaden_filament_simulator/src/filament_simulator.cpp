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
#include "gaden/RunningSimulation.hpp"
#include "gaden/core/Logging.hpp"
#include "gaden/internal/Time.hpp"
#include <visualization_msgs/msg/marker.hpp>

using namespace gaden;
using std::vector;
using visualization_msgs::msg::Marker;

//==============================//
//			MAIN                //
//==============================//
int main(int argc, char** argv)
{
    // Init ROS-NODE
    rclcpp::init(argc, argv);

    gaden::Utils::Time::Stopwatch stopwatch;

    // Create simulator obj and initialize it
    auto sim = std::make_shared<FilamentSimulator>();

    sim->Run();
    if (rclcpp::ok())
        GADEN_INFO_COLOR(fmt::terminal_color::blue, "Filament simulator finished correctly! Ran for {:.2f}s", stopwatch.ellapsed());
}

void FilamentSimulator::Run()
{
    float maxSimTime = getParameter("sim_time", 300.f);

    RunningSimulation::Parameters params;
    EnvironmentConfiguration envConfig;
    std::vector<std::filesystem::path> windFiles;
    std::filesystem::path environmentFile;

    std::filesystem::path projectPath = GadenUtils::getParam<std::string>(shared_from_this(), "projectPath", "");
    if (std::filesystem::exists(projectPath))
    {
        gadenProject.emplace(projectPath);
        GADEN_CHECK_RESULT(gadenProject->ReadDirectory());
        params = gadenProject->simulations.at(getParameter<std::string>("simulationID", "sim"));
        windFiles = GetWindFilePaths(projectPath / "wind");
        environmentFile = projectPath / "OccupancyGrid3D.csv";
    }

    // if we don't have a gaden project directory (using old configurations) read the info from ros parameters
    if (!gadenProject)
    {
        params = {
            .gasType = static_cast<GasType>(getParameter("gas_type", 0)),
            .sourcePosition = Vector3{
                getParameter("source_position_x", 0.0),
                getParameter("source_position_y", 0.0),
                getParameter("source_position_z", 0.0),
            },
            .deltaTime = getParameter("time_step", 0.1f),
            .windIterationDeltaTime = getParameter("wind_time_step", 1.0f),
            .temperature = getParameter("wind_time_step", 298.0f),
            .pressure = getParameter("wind_time_step", 1.0f),
            .filament_ppm_center = getParameter("ppm_filament_center", 20.0f),
            .filament_initial_sigma = getParameter("filament_initial_std", 1.5f),
            .filament_growth_gamma = getParameter("filament_growth_gamma", 10.0f),
            .filament_noise_std = getParameter("filament_noise_std", 0.1f),
            .numFilaments_sec = static_cast<float>(getParameter("num_filaments_sec", 100)),
            .expectedNumIterations = static_cast<size_t>(std::ceil(maxSimTime / params.deltaTime)),
            .windLoop = LoopConfig{.loop = getParameter("allow_looping", false),                   //
                                   .from = static_cast<size_t>(getParameter("loop_from_step", 1)), //
                                   .to = static_cast<size_t>(getParameter("loop_to_step", 100))},
            .saveResults = static_cast<bool>(getParameter("save_results", 1)),
            .saveDeltaTime = getParameter("results_time_step", 0.5f),
            .saveDataDirectory = getParameter<std::string>("results_location", ""),
        };
        windFiles = GetWindFilePaths(getParameter<std::string>("wind_data", ""));
        environmentFile = getParameter<std::string>("occupancy3D_data", "");
    }

    GADEN_CHECK_RESULT(envConfig.environment.ReadFromFile(environmentFile));
    envConfig.windSequence.Initialize(windFiles, envConfig.environment.numCells(), params.windLoop);

    RunningSimulation sim(params, envConfig);

    float runRate = getParameter("runRate", 0); // 0 means as fast as possible
    rclcpp::Rate rate(runRate);
    while (rclcpp::ok() && sim.GetCurrentTime() < maxSimTime)
    {
        sim.AdvanceTimestep();
        const auto& filaments = sim.GetFilaments();
        publishMarkers(filaments);

        if (runRate > 0)
            rate.sleep();
    }
}

void FilamentSimulator::publishMarkers(std::vector<Filament> const& filaments)
{
    static Marker filament_marker;
    static auto publisher = create_publisher<Marker>("filament_visualization", 1);

    // 1. Clean old markers
    filament_marker.points.clear();
    filament_marker.colors.clear();
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
        color.a = 1;
        color.r = 0;
        color.g = 0;
        color.b = 1;

        // Add marker
        filament_marker.points.push_back(point);
        filament_marker.colors.push_back(color);
    }

    // Publish marker of the filaments
    publisher->publish(filament_marker);
}

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