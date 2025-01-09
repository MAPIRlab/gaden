#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <vector>
#include <boost/format.hpp>
#include <gaden_msgs/srv/occupancy.hpp>
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
    struct CADModel
    {
        std::string filepath;
        std_msgs::msg::ColorRGBA color;
        CADModel(const std::string& path, std_msgs::msg::ColorRGBA _color) 
            : filepath(path), color(_color)
        {}

        CADModel(const std::string& path, std::vector<double> _color):
            filepath(path)
        {
            color.r = _color[0];
            color.g = _color[1];
            color.b = _color[2];
            color.a = 1.0;
        }
    };
    std::vector<CADModel> CAD_models;

    // Environment 3D
    std::string occupancy3D_data; // Location of the 3D Occupancy GridMap of the environment
    std::string fixed_frame;      // Frame where to publish the markers
    gaden::Environment environment;

    bool verbose;
    bool wait_preprocessing;
    bool preprocessing_done;

    // Methods
    void loadNodeParameters();
    void loadEnvironment(visualization_msgs::msg::MarkerArray& env_marker);
    int indexFrom3D(int x, int y, int z)
    {
        return gaden::indexFrom3D(gaden::Vector3i(x, y, z), environment.description.dimensions);
    }

    bool occupancyMapServiceCB(gaden_msgs::srv::Occupancy_Request::SharedPtr request,
                               gaden_msgs::srv::Occupancy_Response::SharedPtr response);
    void PreprocessingCB(std_msgs::msg::Bool::SharedPtr b);
    
    static std_msgs::msg::ColorRGBA parseColor(const std::string& str);
};
