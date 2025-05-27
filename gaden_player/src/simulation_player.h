#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <gaden_msgs/msg/gas_in_cell.hpp>
#include <gaden_msgs/srv/gas_position.hpp>
#include <gaden_msgs/srv/wind_position.hpp>

#include <gaden/PlaybackSimulation.hpp>

class Player : public rclcpp::Node
{
public:
    Player();
    void run();

private:
    gaden_msgs::msg::GasInCell GetAllGasesSingleCell(float x, float y, float z, const std::vector<std::string>& gas_types);
    bool GetGasValue_srv(gaden_msgs::srv::GasPosition::Request::SharedPtr req, gaden_msgs::srv::GasPosition::Response::SharedPtr res);
    bool GetWindValue_srv(gaden_msgs::srv::WindPosition::Request::SharedPtr req, gaden_msgs::srv::WindPosition::Response::SharedPtr res);

    void displayCurrentGasDistribution();
    void loadNodeParameters();
    void initSimulations(size_t initialIteration);
    size_t FillMarkerArray(std::vector<geometry_msgs::msg::Point>& marker, std::vector<gaden::Filament> const& filaments);

private:
    std::vector<gaden::PlaybackSimulation::Parameters> params;
    std::vector<gaden::PlaybackSimulation> simulations;
    std::vector<std_msgs::msg::ColorRGBA> gasDisplayColors;

    gaden::EnvironmentConfiguration environmentConfig;
    gaden::LoopConfig loopConfig;

};