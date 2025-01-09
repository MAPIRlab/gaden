/*
 * The only goal of this Node is to display the simulation environment and gas source location in RVIZ.
 *
 * 1. Loads the simulation environment (usually from CFD in the file format .env), and displays it as RVIZ markers.
 * 2. Displays the Gas-Source Location as two cylinders.
 */

#include "environment/environment.h"
#define GADEN_LOGGER_ID "Environment"
#include <gaden_common/Logging.h>
#include <gaden_common/Utils.h>

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

Environment::Environment() : rclcpp::Node("gaden_environment")
{
}

void Environment::run()
{
    using namespace std::placeholders;
    loadNodeParameters();

    // Publishers
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gas_source_pub =
        create_publisher<visualization_msgs::msg::MarkerArray>("source_visualization", 10);
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr environmnet_pub =
        create_publisher<visualization_msgs::msg::MarkerArray>("environment_visualization", 100);
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr environmnet_cad_pub =
        create_publisher<visualization_msgs::msg::MarkerArray>("environment_cad_visualization", 100);

    auto occupancyMapService = create_service<gaden_msgs::srv::Occupancy>("gaden_environment/occupancyMap3D",
                                                                                 std::bind(&Environment::occupancyMapServiceCB, this, _1, _2));
    // Subscribers
    preprocessing_done = false;
    auto sub = create_subscription<std_msgs::msg::Bool>("preprocessing_done", 1, std::bind(&Environment::PreprocessingCB, this, _1));

    // 1. ENVIRONMNET AS CAD MODELS
    //-------------------------------
    visualization_msgs::msg::MarkerArray CAD_model_markers;

    int i = 0;
    for (const CADModel& model : CAD_models)
    {
        // CAD model in Collada (.dae) format
        visualization_msgs::msg::Marker cad;
        cad.header.frame_id = fixed_frame;
        cad.header.stamp = now();
        cad.ns = "part_" + std::to_string(i);
        cad.id = i;
        cad.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        cad.action = visualization_msgs::msg::Marker::ADD;
        cad.mesh_resource = "file://" + model.filepath;
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
        cad.color = model.color;

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

    // 3. GAS SOURCES
    //----------------
    // Generate Gas Source Markers
    // The shape are cylinders from the floor to the given z_size.
    visualization_msgs::msg::MarkerArray gas_source_markers;

    for (int i = 0; i < number_of_sources; i++)
    {
        visualization_msgs::msg::Marker source;
        source.header.frame_id = fixed_frame;
        source.header.stamp = now();
        source.id = i;
        source.ns = "gas_source_visualization";
        source.action = visualization_msgs::msg::Marker::ADD;
        // source.type = visualization_msgs::msg::Marker::CYLINDER;
        source.type = visualization_msgs::msg::Marker::CUBE;

        source.scale.x = gas_source_scale[i];
        source.scale.y = gas_source_scale[i];
        source.scale.z = gas_source_pos_z[i];
        source.color.r = gas_source_color[i][0];
        source.color.g = gas_source_color[i][1];
        source.color.b = gas_source_color[i][2];
        source.color.a = 1.0;
        source.pose.position.x = gas_source_pos_x[i];
        source.pose.position.y = gas_source_pos_y[i];
        source.pose.position.z = gas_source_pos_z[i] / 2;
        source.pose.orientation.x = 0.0;
        source.pose.orientation.y = 0.0;
        source.pose.orientation.z = 1.0;
        source.pose.orientation.w = 1.0;

        // Add Marker to array
        gas_source_markers.markers.push_back(source);
    }

    // Small sleep to allow RVIZ to startup
    sleep(1.0);

    //---------------
    //      LOOP
    //---------------
    rclcpp::Rate r(0.3); // Just to refresh from time to time
    auto shared_this = shared_from_this();
    while (rclcpp::ok())
    {
        // Publish CAD Markers
        environmnet_cad_pub->publish(CAD_model_markers);

        // Publish 3D Occupancy
        if (occupancy3D_data != "")
            environmnet_pub->publish(environment);

        // Publish Gas Sources
        gas_source_pub->publish(gas_source_markers);

        rclcpp::spin_some(shared_this);
        r.sleep();
    }
}

// ===============================//
//      Load Node parameters      //
// ===============================//
void Environment::loadNodeParameters()
{
    verbose = declare_parameter<bool>("verbose", false);
    if (verbose)
        GADEN_INFO("The data provided in the roslaunch file is:");

    wait_preprocessing = declare_parameter<bool>("wait_preprocessing", false);
    if (verbose)
        GADEN_INFO("wait_preprocessing: {}", wait_preprocessing);

    fixed_frame = declare_parameter<std::string>("fixed_frame", "map");
    if (verbose)
        GADEN_INFO("Fixed Frame: {}", fixed_frame.c_str());

    number_of_sources = declare_parameter<int>("number_of_sources", 0);
    if (verbose)
        GADEN_INFO("number_of_sources: {}", number_of_sources);
    gas_source_pos_x.resize(number_of_sources);
    gas_source_pos_y.resize(number_of_sources);
    gas_source_pos_z.resize(number_of_sources);
    gas_source_scale.resize(number_of_sources);
    gas_source_color.resize(number_of_sources);
    for (int i = 0; i < number_of_sources; i++)
    {
        // Get location of soruce for instance (i)
        std::string paramNameX = fmt::format("source_{}_position_x", i);
        std::string paramNameY = fmt::format("source_{}_position_y", i);
        std::string paramNameZ = fmt::format("source_{}_position_z", i);
        std::string scale = fmt::format("source_{}_scale", i);
        std::string color = fmt::format("source_{}_color", i);

        gas_source_pos_x[i] = declare_parameter<double>(paramNameX.c_str(), 0.0);
        gas_source_pos_y[i] = declare_parameter<double>(paramNameY.c_str(), 0.0);
        gas_source_pos_z[i] = declare_parameter<double>(paramNameZ.c_str(), 0.0);
        gas_source_scale[i] = declare_parameter<double>(scale.c_str(), 0.1);
        gas_source_color[i].resize(3);
        gas_source_color[i] = declare_parameter<std::vector<double>>(color.c_str(), {0, 0, 0});

        if (verbose)
            GADEN_INFO("Gas_source({}): pos=[{:.2f} {:.2f} {:.2f}] scale={:.2f}color=[{:.2f} {:.2f} {:.2f}]", i, gas_source_pos_x[i],
                        gas_source_pos_y[i], gas_source_pos_z[i], gas_source_scale[i], gas_source_color[i][0], gas_source_color[i][1],
                        gas_source_color[i][2]);
    }

    // CAD MODELS
    //-------------
    // CAD model files
    auto CAD_strings = getParam<std::vector<std::string>>(shared_from_this(), "CAD_models", std::vector<std::string>{});

    if(CAD_strings.empty()) //try the old style, with numbered parameters instead of a single list
    {
        int i = 0;
        while (true)
        {
            std::string param_name = fmt::format("CAD_{}", i);
            std::string paramColor = fmt::format("CAD_{}_color", i);
            std::string model = getParam<std::string>(shared_from_this(), param_name, "");
            auto color = getParam<std::vector<double>>(shared_from_this(), paramColor.c_str(), {0, 0, 0});
            if (model != "")
            {
                CAD_models.emplace_back(model, color);
            }
            else
                break;
            i++;
        }
        if(i>0)
            GADEN_WARN("Specifying models through numbered parameters is deprecated. You should use a single list parameter instead (see test_env for examples)");
    }
    else
    {
        std_msgs::msg::ColorRGBA lastColor;
        //set default color
        {
            lastColor.r = 1.0;
            lastColor.g = 1.0;
            lastColor.b = 1.0;
            lastColor.a = 1.0;
        }
    
        for(const std::string& str : CAD_strings)
        {
            if(str.find("!color") != std::string::npos)
                lastColor = parseColor(str);
            else
                CAD_models.emplace_back(str, lastColor);
        }
    }

    if (verbose)
        GADEN_INFO("number_of_CAD: {}", CAD_models.size());

    // Occupancy 3D gridmap
    //---------------------
    occupancy3D_data = declare_parameter<std::string>("occupancy3D_data", "");
    if (verbose)
        GADEN_INFO("Occupancy3D file location: {}", occupancy3D_data.c_str());
}

//=========================//
// PreProcessing CallBack  //
//=========================//
void Environment::PreprocessingCB(std_msgs::msg::Bool::SharedPtr b)
{
    preprocessing_done = true;
}

/* Load environment from 3DOccupancy.csv GridMap
 * Loads the environment file containing a description of the simulated environment in the CFD (for the estimation of the wind flows), and displays
 * it. As a general rule, environment files set a value of "0" for a free cell, "1" for a ocuppiedd cell and "2" for outlet. This function creates a
 * cube marker for every occupied cell, with the corresponding dimensions
 */
void Environment::loadEnvironment(visualization_msgs::msg::MarkerArray& env_marker)
{
    // Wait for the GADEN_preprocessin node to finish?
    if (wait_preprocessing)
    {
        rclcpp::Rate r(2);
        auto shared_this = shared_from_this();
        while (rclcpp::ok() && !preprocessing_done)
        {
            r.sleep();
            rclcpp::spin_some(shared_this);
            if (verbose)
                GADEN_INFO("Waiting for node GADEN_preprocessing to end.");
        }
    }

    gaden::ReadResult result = gaden::readEnvFile(occupancy3D_data, environment);
    if (result == gaden::ReadResult::NO_FILE)
    {
        GADEN_ERROR("No occupancy file provided to environment node!");
        return;
    }
    else if (result == gaden::ReadResult::READING_FAILED)
    {
        GADEN_ERROR("Something went wrong while parsing the file!");
    }

    for (int i = 0; i < environment.description.dimensions.x; i++)
    {
        for (int j = 0; j < environment.description.dimensions.y; j++)
        {
            for (int k = 0; k < environment.description.dimensions.z; k++)
            {
                // Color
                if (!environment.Env[indexFrom3D(i, j, k)])
                {
                    // Add a new cube marker for this occupied cell
                    visualization_msgs::msg::Marker new_marker;
                    new_marker.header.frame_id = fixed_frame;
                    new_marker.header.stamp = now();
                    new_marker.ns = "environment_visualization";
                    new_marker.id = indexFrom3D(i, j, k); // unique identifier
                    new_marker.type = visualization_msgs::msg::Marker::CUBE;
                    new_marker.action = visualization_msgs::msg::Marker::ADD;

                    // Center of the cell
                    new_marker.pose.position.x = environment.description.min_coord.x + ((i + 0.5) * environment.description.cell_size);
                    new_marker.pose.position.y = environment.description.min_coord.y + ((j + 0.5) * environment.description.cell_size);
                    new_marker.pose.position.z = environment.description.min_coord.z + ((k + 0.5) * environment.description.cell_size);
                    new_marker.pose.orientation.x = 0.0;
                    new_marker.pose.orientation.y = 0.0;
                    new_marker.pose.orientation.z = 0.0;
                    new_marker.pose.orientation.w = 1.0;

                    // Size of the cell
                    new_marker.scale.x = environment.description.cell_size;
                    new_marker.scale.y = environment.description.cell_size;
                    new_marker.scale.z = environment.description.cell_size;

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
    response->origin.x = environment.description.min_coord.x;
    response->origin.y = environment.description.min_coord.y;
    response->origin.z = environment.description.min_coord.z;

    response->num_cells_x = environment.description.dimensions.x;
    response->num_cells_y = environment.description.dimensions.y;
    response->num_cells_z = environment.description.dimensions.z;

    response->occupancy = environment.Env;
    response->resolution = environment.description.cell_size;

    return true;
}


std_msgs::msg::ColorRGBA Environment::parseColor(const std::string& str)
{
    std_msgs::msg::ColorRGBA color;
    color.a = 1.0;
    
    std::stringstream ss(str);
    ss >> std::skipws;

    ss.ignore(256, '[');
    ss >> color.r;
    ss.ignore(256, ',');
    ss >> color.g;
    ss.ignore(256, ',');
    ss >> color.b;

    ss.ignore(256, ',');
    if(!ss.eof())
        ss >> color.a;
    
    return color;
}