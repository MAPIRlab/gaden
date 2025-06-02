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
    float maxSimTime = parameter("sim_time", 300.f);

    RunningSimulation::Parameters params{
        .gasType = static_cast<GasType>(parameter("gas_type", 0)),
        .sourcePosition = Vector3{
            parameter("source_position_x", 0.0),
            parameter("source_position_y", 0.0),
            parameter("source_position_z", 0.0),
        },
        .deltaTime = parameter("time_step", 0.1f),
        .windIterationDeltaTime = parameter("wind_time_step", 1.0f),
        .temperature = parameter("wind_time_step", 298.0f),
        .pressure = parameter("wind_time_step", 1.0f),
        .filament_ppm_center = parameter("ppm_filament_center", 20.0f),
        .filament_initial_sigma = parameter("filament_initial_std", 1.5f),
        .filament_growth_gamma = parameter("filament_growth_gamma", 10.0f),
        .filament_noise_std = parameter("filament_noise_std", 0.1f),
        .numFilaments_sec = static_cast<float>(parameter("num_filaments_sec", 100)),
        .expectedNumIterations = static_cast<size_t>(std::ceil(maxSimTime / params.deltaTime)),
        .windLoop = LoopConfig{.loop = parameter("allow_looping", false),                   //
                               .from = static_cast<size_t>(parameter("loop_from_step", 1)), //
                               .to = static_cast<size_t>(parameter("loop_to_step", 100))},
        .saveResults = static_cast<bool>(parameter("save_results", 1)),
        .saveDeltaTime = parameter("results_time_step", 0.5f),
        .saveDataDirectory = parameter<std::string>("results_location", ""),
        .simulationID = parameter<std::string>("simulationID", "sim"),
    };

    EnvironmentConfiguration envConfig;
    GADEN_CHECK_RESULT(envConfig.environment.ReadFromFile(parameter<std::string>("occupancy3D_data", "")));
    envConfig.windSequence.Initialize(GetWindFilePaths(), envConfig.environment.numCells(), params.windLoop);

    RunningSimulation sim(params, envConfig);

    float runRate = parameter("runRate", 0);
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

std::vector<std::filesystem::path> FilamentSimulator::GetWindFilePaths()
{
    std::string windFilesLocation = declare_parameter<std::string>("wind_data", "");
    std::vector<std::filesystem::path> paths;
    GADEN_INFO_COLOR(fmt::terminal_color::blue, "Parameter 'wind_data': '{}'", windFilesLocation);

    // post 3.0
    GADEN_INFO("Trying to load wind files with pattern '{}_i'", windFilesLocation);
    paths = GadenUtils::GetWindFiles([](std::string const& path, size_t idx)
                                     {
                                         return fmt::format("{}_{}", path, idx);
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