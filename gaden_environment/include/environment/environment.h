#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <gaden_msgs/srv/occupancy.hpp>
#include <gaden/Environment.hpp>
#include <gaden/EnvironmentConfigMetadata.hpp>
#include <gaden_common/Utils.hpp>

class Environment : public rclcpp::Node
{
public:
    Environment();
    void run();

private:

    // CAD models
    std::vector<gaden::Model3D> CAD_models;

    // Environment 3D
    std::string occupancy3D_data; // Location of the 3D Occupancy GridMap of the environment
    std::string fixed_frame;      // Frame where to publish the markers
    gaden::Environment environment;

    std::optional<gaden::EnvironmentConfigMetadata> gadenProject;

    // Methods
    void loadNodeParameters();
    void loadGadenProject();
    void loadEnvironment(visualization_msgs::msg::MarkerArray& env_marker);

    bool occupancyMapServiceCB(gaden_msgs::srv::Occupancy_Request::SharedPtr request,
                               gaden_msgs::srv::Occupancy_Response::SharedPtr response);
    

    template <typename T>
    T getParameter(std::string const& name, T defaultValue)
    {
        return GadenUtils::getParam<T>(shared_from_this(), name, defaultValue);
    }
};
