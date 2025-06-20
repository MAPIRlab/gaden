#include <gaden/EnvironmentConfigMetadata.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <gaden_msgs/msg/gas_in_cell.hpp>
#include <gaden_msgs/srv/gas_position.hpp>
#include <gaden_msgs/srv/wind_position.hpp>

#include <gaden/PlaybackScene.hpp>

class Player : public rclcpp::Node
{
public:
    Player();
    void run();

private:
    gaden_msgs::msg::GasInCell GetAllGasesSingleCell(float x, float y, float z, const std::vector<gaden::GasType>& gas_types);
    bool GetGasValue_srv(gaden_msgs::srv::GasPosition::Request::SharedPtr req, gaden_msgs::srv::GasPosition::Response::SharedPtr res);
    bool GetWindValue_srv(gaden_msgs::srv::WindPosition::Request::SharedPtr req, gaden_msgs::srv::WindPosition::Response::SharedPtr res);

    void displayCurrentGasDistribution();
    void loadNodeParameters();
    void loadGadenProject();
    void initSimulations();
    size_t FillMarkerArray(std::vector<geometry_msgs::msg::Point>& marker, std::vector<gaden::Filament> const& filaments);

private:
    gaden::PlaybackSceneMetadata playbackMetadata;
    std::optional<gaden::PlaybackScene> playbackScene;

    gaden::EnvironmentConfiguration environmentConfig;
    std::optional<gaden::EnvironmentConfigMetadata> gadenProject;
};