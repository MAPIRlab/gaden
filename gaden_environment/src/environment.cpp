/*
 * The only goal of this Node is to display the simulation environment and gas source location in RVIZ.
 *
 * 1. Loads the simulation environment (usually from CFD in the file format .env), and displays it as RVIZ markers.
 * 2. Displays the Gas-Source Location as two cylinders.
 */

#include "environment/environment.h"

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
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr gas_source_pub = create_publisher<visualization_msgs::msg::MarkerArray>("source_visualization", 10);
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr environmnet_pub = create_publisher<visualization_msgs::msg::MarkerArray>("environment_visualization", 100);
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr environmnet_cad_pub = create_publisher<visualization_msgs::msg::MarkerArray>("environment_cad_visualization", 100);

    auto occupancyMapService = create_service<gaden_environment::srv::Occupancy>("gaden_environment/occupancyMap3D", std::bind(&Environment::occupancyMapServiceCB, this, _1, _2));
    // Subscribers
    preprocessing_done = false;
    auto sub = create_subscription<std_msgs::msg::Bool>("preprocessing_done", 1, std::bind(&Environment::PreprocessingCB, this, _1));

    // 1. ENVIRONMNET AS CAD MODELS
    //-------------------------------
    visualization_msgs::msg::MarkerArray CAD_model_markers;

    for (int i = 0; i < number_of_CAD; i++)
    {
        // CAD model in Collada (.dae) format
        visualization_msgs::msg::Marker cad;
        cad.header.frame_id = fixed_frame;
        cad.header.stamp = now();
        cad.ns = "part_" + std::to_string(i);
        cad.id = i;
        cad.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        cad.action = visualization_msgs::msg::Marker::ADD;
        cad.mesh_resource = "file://" + CAD_models[i];
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
        cad.color.r = CAD_color[i][0];
        cad.color.g = CAD_color[i][1];
        cad.color.b = CAD_color[i][2];
        cad.color.a = 1.0;

        // Add Marker to array
        CAD_model_markers.markers.push_back(cad);
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
        RCLCPP_INFO(get_logger(), "[env] The data provided in the roslaunch file is:");

    wait_preprocessing = declare_parameter<bool>("wait_preprocessing", false);
    if (verbose)
        RCLCPP_INFO(get_logger(), "[env] wait_preprocessing: %u", wait_preprocessing);

    fixed_frame = declare_parameter<std::string>("fixed_frame", "map");
    if (verbose)
        RCLCPP_INFO(get_logger(), "[env] Fixed Frame: %s", fixed_frame.c_str());

    number_of_sources = declare_parameter<int>("number_of_sources", 0);
    if (verbose)
        RCLCPP_INFO(get_logger(), "[env] number_of_sources: %i", number_of_sources);
    gas_source_pos_x.resize(number_of_sources);
    gas_source_pos_y.resize(number_of_sources);
    gas_source_pos_z.resize(number_of_sources);
    gas_source_scale.resize(number_of_sources);
    gas_source_color.resize(number_of_sources);
    for (int i = 0; i < number_of_sources; i++)
    {
        // Get location of soruce for instance (i)
        std::string paramNameX = boost::str(boost::format("source_%i_position_x") % i);
        std::string paramNameY = boost::str(boost::format("source_%i_position_y") % i);
        std::string paramNameZ = boost::str(boost::format("source_%i_position_z") % i);
        std::string scale = boost::str(boost::format("source_%i_scale") % i);
        std::string color = boost::str(boost::format("source_%i_color") % i);

        gas_source_pos_x[i] = declare_parameter<double>(paramNameX.c_str(), 0.0);
        gas_source_pos_y[i] = declare_parameter<double>(paramNameY.c_str(), 0.0);
        gas_source_pos_z[i] = declare_parameter<double>(paramNameZ.c_str(), 0.0);
        gas_source_scale[i] = declare_parameter<double>(scale.c_str(), 0.1);
        gas_source_color[i].resize(3);
        gas_source_color[i] = declare_parameter<std::vector<double>>(color.c_str(), { 0, 0, 0 });

        if (verbose)
            RCLCPP_INFO(get_logger(), "[env] Gas_source(%i): pos=[%0.2f %0.2f %0.2f] scale=%.2f color=[%0.2f %0.2f %0.2f]",
                i, gas_source_pos_x[i], gas_source_pos_y[i], gas_source_pos_z[i],
                gas_source_scale[i],
                gas_source_color[i][0], gas_source_color[i][1], gas_source_color[i][2]);
    }

    // CAD MODELS
    //-------------
    // CAD model files
    number_of_CAD = declare_parameter<int>("number_of_CAD", 0);
    if (verbose)
        RCLCPP_INFO(get_logger(), "[env] number_of_CAD: %i", number_of_CAD);

    CAD_models.resize(number_of_CAD);
    CAD_color.resize(number_of_CAD);
    for (int i = 0; i < number_of_CAD; i++)
    {
        // Get location of CAD file for instance (i)
        std::string paramName = boost::str(boost::format("CAD_%i") % i);
        std::string paramColor = boost::str(boost::format("CAD_%i_color") % i);

        CAD_models[i] = declare_parameter<std::string>(paramName.c_str(), "");
        CAD_color[i].resize(3);
        CAD_color[i] = declare_parameter<std::vector<double>>(paramColor.c_str(), { 0, 0, 0 });
        if (verbose)
            RCLCPP_INFO(get_logger(), "[env] CAD_models(%i): %s", i, CAD_models[i].c_str());
    }

    // Occupancy 3D gridmap
    //---------------------
    occupancy3D_data = declare_parameter<std::string>("occupancy3D_data", "");
    if (verbose)
        RCLCPP_INFO(get_logger(), "[env] Occupancy3D file location: %s", occupancy3D_data.c_str());
}

//=========================//
// PreProcessing CallBack  //
//=========================//
void Environment::PreprocessingCB(std_msgs::msg::Bool::SharedPtr b)
{
    preprocessing_done = true;
}

/* Load environment from 3DOccupancy.csv GridMap
 * Loads the environment file containing a description of the simulated environment in the CFD (for the estimation of the wind flows), and displays it.
 * As a general rule, environment files set a value of "0" for a free cell, "1" for a ocuppiedd cell and "2" for outlet.
 * This function creates a cube marker for every occupied cell, with the corresponding dimensions
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
                RCLCPP_INFO(get_logger(), "[environment] Waiting for node GADEN_preprocessing to end.");
        }
    }

    Gaden::ReadResult result = Gaden::readEnvFile(occupancy3D_data, env_desc);
    if (result == Gaden::ReadResult::NO_FILE)
    {
        RCLCPP_ERROR(get_logger(), "No occupancy file provided to environment node!");
        return;
    }
    else if (result == Gaden::ReadResult::READING_FAILED)
    {
        RCLCPP_ERROR(get_logger(), "Something went wrong while parsing the file!");
    }

    for (int i = 0; i < env_desc.num_cells.x; i++)
    {
        for (int j = 0; j < env_desc.num_cells.y; j++)
        {
            for (int k = 0; k < env_desc.num_cells.z; k++)
            {
                // Color
                if (!env_desc.Env[indexFrom3D(i, j, k)])
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
                    new_marker.pose.position.x = env_desc.min_coord.x + ((i + 0.5) * env_desc.cell_size);
                    new_marker.pose.position.y = env_desc.min_coord.y + ((j + 0.5) * env_desc.cell_size);
                    new_marker.pose.position.z = env_desc.min_coord.z + ((k + 0.5) * env_desc.cell_size);
                    new_marker.pose.orientation.x = 0.0;
                    new_marker.pose.orientation.y = 0.0;
                    new_marker.pose.orientation.z = 0.0;
                    new_marker.pose.orientation.w = 1.0;

                    // Size of the cell
                    new_marker.scale.x = env_desc.cell_size;
                    new_marker.scale.y = env_desc.cell_size;
                    new_marker.scale.z = env_desc.cell_size;

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

bool Environment::occupancyMapServiceCB(gaden_environment::srv::Occupancy_Request::SharedPtr request, gaden_environment::srv::Occupancy_Response::SharedPtr response)
{
    response->origin.x = env_desc.min_coord.x;
    response->origin.y = env_desc.min_coord.y;
    response->origin.z = env_desc.min_coord.z;

    response->num_cells_x = env_desc.num_cells.x;
    response->num_cells_y = env_desc.num_cells.y;
    response->num_cells_z = env_desc.num_cells.z;

    response->occupancy = env_desc.Env;
    response->resolution = env_desc.cell_size;

    return true;
}
