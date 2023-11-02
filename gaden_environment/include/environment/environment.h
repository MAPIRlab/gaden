#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <vector>
#include <fstream>
#include <boost/format.hpp>
#include <gaden_environment/srv/occupancy.hpp>
#include <gaden_common/ReadEnvironment.h>
class Environment : public rclcpp::Node
{
public:
    Environment();
    void run();

private:
    // Gas Sources
    int number_of_sources;
    std::vector<double> gas_source_pos_x;
    std::vector<double> gas_source_pos_y;
    std::vector<double> gas_source_pos_z;
    std::vector<double> gas_source_scale;
    std::vector<std::vector<double>> gas_source_color;

    // CAD models
    int number_of_CAD;
    std::vector<std::string> CAD_models;
    std::vector<std::vector<double>> CAD_color;

    // Environment 3D
    std::string occupancy3D_data; // Location of the 3D Occupancy GridMap of the environment
    std::string fixed_frame;      // Frame where to publish the markers
    Gaden::Environment environment;

    bool verbose;
    bool wait_preprocessing;
    bool preprocessing_done;

    // Methods
    void loadNodeParameters();
    void loadEnvironment(visualization_msgs::msg::MarkerArray& env_marker);
    int indexFrom3D(int x, int y, int z)
    {
        return Gaden::indexFrom3D(Gaden::Vector3i(x, y, z), environment.description.num_cells);
    }

    bool occupancyMapServiceCB(gaden_environment::srv::Occupancy_Request::SharedPtr request,
                               gaden_environment::srv::Occupancy_Response::SharedPtr response);
    void PreprocessingCB(std_msgs::msg::Bool::SharedPtr b);
};
