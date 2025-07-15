#pragma once

#include <gaden/EnvironmentConfigMetadata.hpp>
#include <gaden/datatypes/Filament.hpp>
#include <gaden_common/Utils.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class FilamentSimulator : public rclcpp::Node
{
public:
    FilamentSimulator()
        : Node("FilamentSimlator")
    {}
    void Run();

private:
    void AddAirflowDisturbance(const geometry_msgs::msg::PointStamped::SharedPtr rotorPosition);

    void publishMarkers(std::vector<gaden::Filament> const& filaments);
    template <typename T>
    T getParameter(std::string const& name, T defaultValue)
    {
        return GadenUtils::getParam<T>(shared_from_this(), name, defaultValue);
    }

    std::vector<std::filesystem::path> GetWindFilePaths(std::filesystem::path const& windFilesLocation);

private:
    std::optional<gaden::EnvironmentConfigMetadata> gadenProject;
    std::optional<gaden::RunningSimulation> sim;

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rotorPositionSub;
    rclcpp::Publisher<Marker>::SharedPtr gasPublisher;
    rclcpp::Publisher<MarkerArray>::SharedPtr sourcePublisher;
};
