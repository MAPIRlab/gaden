/*
 * The only goal of this Node is to display the simulation environment and gas source location in RVIZ.
 *
 * 1. Loads the simulation environment from model files
 * 2. Displays the Gas-Source Location as two cylinders.
 */

#define GADEN_LOGGER_ID "Environment"
#include "environment/environment.h"
#include <gaden/core/Assertions.hpp>
#include <gaden/core/Logging.hpp>
#include <gaden/internal/STL.hpp>

// ===============================//
//              MAIN              //
// ===============================//
int main(int argc, char** argv)
{
    // Init
    rclcpp::init(argc, argv);

    std::shared_ptr<Environment> environment = std::make_shared<Environment>();
    // Load Parameters
    environment->run();
}

Environment::Environment()
    : rclcpp::Node("gaden_environment")
{
}

void Environment::run()
{
    using namespace std::placeholders;
    fixed_frame = getParameter<std::string>("fixed_frame", "map");

    std::filesystem::path projectPath = getParameter<std::string>("projectPath", "");
    if (std::filesystem::exists(projectPath))
    {
        gadenProject.emplace(projectPath);
        GADEN_CHECK_RESULT(gadenProject->ReadDirectory());
    }

    if (!gadenProject)
        loadNodeParameters();
    else
        loadGadenProject();

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr environment_pub =
        create_publisher<visualization_msgs::msg::MarkerArray>("environment_visualization", rclcpp::QoS(1).transient_local());
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr environment_cad_pub =
        create_publisher<visualization_msgs::msg::MarkerArray>("environment_cad_visualization", rclcpp::QoS(1).transient_local());

    auto occupancyMapService = create_service<gaden_msgs::srv::Occupancy>("gaden_environment/occupancyMap3D",
                                                                          std::bind(&Environment::occupancyMapServiceCB, this, _1, _2));

    // 1. ENVIRONMNET AS CAD MODELS
    //-------------------------------
    visualization_msgs::msg::MarkerArray CAD_model_markers;

    int i = 0;
    for (const gaden::Model3D& model : CAD_models)
    {
        if (model.path.extension() == ".stl" && gaden::isASCII(model.path))
        {
            GADEN_WARN("Rviz does not display ASCII .stl files. Please convert '{}' to binary .stl", model.path);
            continue;
        }

        // CAD model in Collada (.dae) format
        visualization_msgs::msg::Marker cad;
        cad.header.frame_id = fixed_frame;
        cad.header.stamp = now();
        cad.ns = "part_" + std::to_string(i);
        cad.id = i;
        cad.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        cad.action = visualization_msgs::msg::Marker::ADD;
        cad.mesh_resource = "file://" + std::filesystem::canonical(model.path).string();
        cad.scale.x = 1.0;
        cad.scale.y = 1.0;
        cad.scale.z = 1.0;
        cad.pose.position.x = 0.0; // CAD models have the object pose within the file!
        cad.pose.position.y = 0.0;
        cad.pose.position.z = 0.0;
        cad.pose.orientation.x = 0.0;
        cad.pose.orientation.y = 0.0;
        cad.pose.orientation.z = 0.0;
        cad.pose.orientation.w = 1.0;

        // Color (Collada has no color)
        cad.color.r = model.color.r;
        cad.color.g = model.color.g;
        cad.color.b = model.color.b;
        cad.color.a = model.color.a;

        // Add Marker to array
        CAD_model_markers.markers.push_back(cad);
        i++;
    }

    // 2. ENVIRONMNET AS Occupancy3D file
    //------------------------------------
    // Display Environment as an array of Cube markers (Rviz)
    visualization_msgs::msg::MarkerArray environment;
    if (occupancy3D_data != "")
        loadEnvironment(environment);

    // Publish CAD Markers

    // Publish 3D Occupancy
    if (occupancy3D_data != "")
        environment_pub->publish(environment);

    //---------------
    //      LOOP
    //---------------
    rclcpp::Rate r(10); // Just to refresh from time to time
    auto shared_this = shared_from_this();
    while (rclcpp::ok())
    {
        environment_cad_pub->publish(CAD_model_markers);
        rclcpp::spin_some(shared_this);
        r.sleep();
    }
}

// ===============================//
//      Load Node parameters      //
// ===============================//
void Environment::loadNodeParameters()
{
    GadenUtils::OldProjectWarning();

    // CAD MODELS
    //-------------
    // CAD model files
    auto CAD_strings = GadenUtils::getParam<std::vector<std::string>>(shared_from_this(), "CAD_models", std::vector<std::string>{});

    if (CAD_strings.empty()) // try the old style, with numbered parameters instead of a single list
    {
        int i = 0;
        while (true)
        {
            std::string param_name = fmt::format("CAD_{}", i);
            std::string paramColor = fmt::format("CAD_{}_color", i);
            std::string model = getParameter<std::string>(param_name, "");
            auto colorParam = getParameter<std::vector<double>>(paramColor.c_str(), {0, 0, 0});
            if (model != "")
            {
                gaden::Color color;
                color.r = colorParam[0];
                color.g = colorParam[1];
                color.b = colorParam[2];
                CAD_models.push_back(gaden::Model3D{.path = model, .color = color});
            }
            else
                break;
            i++;
        }
        if (i > 0)
            GADEN_WARN("Specifying models through numbered parameters is deprecated. You should use a single list parameter instead (see test_env for examples)");
    }
    else
    {
        gaden::Color lastColor;
        // set default color
        {
            lastColor.r = 1.0;
            lastColor.g = 1.0;
            lastColor.b = 1.0;
            lastColor.a = 1.0;
        }

        for (const std::string& str : CAD_strings)
        {
            if (str.find("!color") != std::string::npos)
                lastColor = gaden::Color::Parse(str);
            else
                CAD_models.push_back({str, lastColor});
        }
    }

    // Occupancy 3D gridmap
    //---------------------
    occupancy3D_data = getParameter<std::string>("occupancy3D_data", "");
}

void Environment::loadGadenProject()
{
    std::copy(gadenProject->envModels.begin(), gadenProject->envModels.end(), std::back_inserter(CAD_models));
    std::copy(gadenProject->outletModels.begin(), gadenProject->outletModels.end(), std::back_inserter(CAD_models));

    occupancy3D_data = getParameter<std::string>("projectPath", "") + "/OccupancyGrid3D.csv";
}

/* Load environment from 3DOccupancy.csv GridMap
 * Loads the environment file containing a description of the simulated environment in the CFD (for the estimation of the wind flows), and displays
 * it. As a general rule, environment files set a value of "0" for a free cell, "1" for a ocuppiedd cell and "2" for outlet. This function creates a
 * cube marker for every occupied cell, with the corresponding dimensions
 */
void Environment::loadEnvironment(visualization_msgs::msg::MarkerArray& env_marker)
{
    GADEN_CHECK_RESULT(environment.ReadFromFile(occupancy3D_data));

    for (int i = 0; i < environment.description.dimensions.x; i++)
    {
        for (int j = 0; j < environment.description.dimensions.y; j++)
        {
            for (int k = 0; k < environment.description.dimensions.z; k++)
            {
                // Color
                size_t idx = environment.indexFrom3D(gaden::Vector3i(i, j, k));
                if (environment.cells.at(idx) == gaden::Environment::CellState::Obstacle)
                {
                    // Add a new cube marker for this occupied cell
                    visualization_msgs::msg::Marker new_marker;
                    new_marker.header.frame_id = fixed_frame;
                    new_marker.header.stamp = now();
                    new_marker.ns = "environment_visualization";
                    new_marker.id = idx; // unique identifier
                    new_marker.type = visualization_msgs::msg::Marker::CUBE;
                    new_marker.action = visualization_msgs::msg::Marker::ADD;

                    // Center of the cell
                    new_marker.pose.position.x = environment.description.minCoord.x + ((i + 0.5) * environment.description.cellSize);
                    new_marker.pose.position.y = environment.description.minCoord.y + ((j + 0.5) * environment.description.cellSize);
                    new_marker.pose.position.z = environment.description.minCoord.z + ((k + 0.5) * environment.description.cellSize);
                    new_marker.pose.orientation.x = 0.0;
                    new_marker.pose.orientation.y = 0.0;
                    new_marker.pose.orientation.z = 0.0;
                    new_marker.pose.orientation.w = 1.0;

                    // Size of the cell
                    new_marker.scale.x = environment.description.cellSize;
                    new_marker.scale.y = environment.description.cellSize;
                    new_marker.scale.z = environment.description.cellSize;

                    new_marker.color.r = 0.9f;
                    new_marker.color.g = 0.1f;
                    new_marker.color.b = 0.1f;
                    new_marker.color.a = 1.0;
                    env_marker.markers.push_back(new_marker);
                }
            }
        }
    }
}

bool Environment::occupancyMapServiceCB(gaden_msgs::srv::Occupancy_Request::SharedPtr request,
                                        gaden_msgs::srv::Occupancy_Response::SharedPtr response)
{
    response->origin.x = environment.description.minCoord.x;
    response->origin.y = environment.description.minCoord.y;
    response->origin.z = environment.description.minCoord.z;

    response->num_cells_x = environment.description.dimensions.x;
    response->num_cells_y = environment.description.dimensions.y;
    response->num_cells_z = environment.description.dimensions.z;

    response->occupancy.resize(environment.cells.size());
#pragma omp parallel for
    for (size_t i = 0; i < environment.cells.size(); i++)
        response->occupancy.at(i) = static_cast<uint8_t>(environment.cells.at(i));

    response->resolution = environment.description.cellSize;

    return true;
}
