#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>

#include <tf2/buffer_core.h>

#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <olfaction_msgs/msg/anemometer.hpp>
#include <gaden_msgs/srv/wind_position.hpp>

#include <cstdlib>
#include <math.h>

class SimulatedAnemometer : public rclcpp::Node
{
public:
    SimulatedAnemometer();
    void run();

private:
    // Sensor Parameters
    std::string input_sensor_frame;
    std::string input_fixed_frame;
    double noise_std;
    double frequency;
    bool use_map_ref_system;

    // Vars
    bool first_reading = true;

    // functions:
    void loadNodeParameters();
};