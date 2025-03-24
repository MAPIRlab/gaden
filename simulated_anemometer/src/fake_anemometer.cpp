
#include "fake_anemometer.h"

#include <fmt/format.h>
#include <random>
#include <stdlib.h> /* srand, rand */
#include <time.h>

typedef std::normal_distribution<double> NormalDistribution;
typedef std::mt19937 RandomGenerator;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    srand(time(NULL));

    std::shared_ptr<SimulatedAnemometer> anemometer = std::make_shared<SimulatedAnemometer>();
    anemometer->run();

    return 0;
}

SimulatedAnemometer::SimulatedAnemometer()
    : rclcpp::Node("Simulated_anemometer")
{}

void SimulatedAnemometer::run()
{
    auto shared_this = shared_from_this();
    // Read parameters
    loadNodeParameters();

    // Publishers
    std::string topic = declare_parameter<std::string>("topic", fmt::format("{}/{}", get_fully_qualified_name(), "WindSensor_reading"));
    auto sensor_read_pub = create_publisher<olfaction_msgs::msg::Anemometer>(topic, 500);
    auto marker_pub = create_publisher<visualization_msgs::msg::Marker>(fmt::format("{}/{}", get_fully_qualified_name(), "WindSensor_display"), 100);

    // Service to request wind values to simulator
    auto playerClient = create_client<gaden_msgs::srv::WindPosition>("/wind_value");

    // Init Visualization data (marker)
    //----------------------------------------------------------------
    // sensor = sphere
    // conector = stick from the floor to the sensor
    visualization_msgs::msg::Marker sensor;
    visualization_msgs::msg::Marker connector;
    visualization_msgs::msg::Marker connector_inv;
    visualization_msgs::msg::Marker wind_point;
    visualization_msgs::msg::Marker wind_point_inv;

    sensor.header.frame_id = input_fixed_frame;
    sensor.ns = "sensor_visualization";
    sensor.action = visualization_msgs::msg::Marker::ADD;
    sensor.type = visualization_msgs::msg::Marker::SPHERE;
    sensor.id = 0;
    sensor.scale.x = 0.1;
    sensor.scale.y = 0.1;
    sensor.scale.z = 0.1;
    sensor.color.r = 0.0f;
    sensor.color.g = 0.0f;
    sensor.color.b = 1.0f;
    sensor.color.a = 1.0;

    connector.header.frame_id = input_fixed_frame;
    connector.ns = "sensor_visualization";
    connector.action = visualization_msgs::msg::Marker::ADD;
    connector.type = visualization_msgs::msg::Marker::CYLINDER;
    connector.id = 1;
    connector.scale.x = 0.1;
    connector.scale.y = 0.1;
    connector.color.a = 1.0;
    connector.color.r = 1.0f;
    connector.color.b = 1.0f;
    connector.color.g = 1.0f;

    // Init Marker: arrow to display the wind direction measured.
    wind_point.header.frame_id = input_sensor_frame;
    wind_point.action = visualization_msgs::msg::Marker::ADD;
    wind_point.ns = "measured_wind";
    wind_point.type = visualization_msgs::msg::Marker::ARROW;

    // Init Marker: arrow to display the inverted wind direction measured.
    wind_point_inv.header.frame_id = input_sensor_frame;
    wind_point_inv.action = visualization_msgs::msg::Marker::ADD;
    wind_point_inv.ns = "measured_wind_inverted";
    wind_point_inv.type = visualization_msgs::msg::Marker::ARROW;

    auto tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    auto listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    while (rclcpp::ok() && !playerClient->wait_for_service(std::chrono::seconds(5)))
        RCLCPP_INFO(get_logger(), "WAITING FOR GADEN_PLAYER SERVICE");

    rclcpp::Rate r(frequency);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(shared_this);

        geometry_msgs::msg::TransformStamped anemometer_transform_map;
        bool know_sensor_pose = true;

        // Get pose of the sensor in the /map reference
        try
        {
            anemometer_transform_map = tf_buffer->lookupTransform(input_fixed_frame, input_sensor_frame, rclcpp::Time(0));
        }
        catch (tf2::TransformException& ex)
        {
            RCLCPP_ERROR(get_logger(), "%s", ex.what());
            know_sensor_pose = false;

            rclcpp::sleep_for(std::chrono::seconds(1));
        }

        if (know_sensor_pose)
        {
            // Get Wind vectors (u,v,w) at current position
            // Service request to the simulator
            auto request = std::make_shared<gaden_msgs::srv::WindPosition::Request>();
            request->x.push_back(anemometer_transform_map.transform.translation.x);
            request->y.push_back(anemometer_transform_map.transform.translation.y);
            request->z.push_back(anemometer_transform_map.transform.translation.z);

            float u, v, w;
            olfaction_msgs::msg::Anemometer anemo_msg;

            auto result = playerClient->async_send_request(request);
            if (rclcpp::spin_until_future_complete(shared_this, result, std::chrono::seconds(1)) == rclcpp::FutureReturnCode::SUCCESS)
            {
                auto response = result.get();

                // GT Wind vector Value (u,v,w)[m/s]
                // From OpenFoam this is the DownWind direction in the map
                u = (float)response->u[0];
                v = (float)response->v[0];
                w = (float)response->w[0];

                double wind_speed = u * u + v * v; // ignore the w component, because the anemometer is only 2D
                double wind_direction;

                if (!use_map_ref_system)
                {
                    // (IMPORTANT) Follow standards on wind measurement (real anemometers):
                    // return the upwind direction in the anemometer reference system
                    // range [-pi,pi]
                    // positive to the right, negative to the left (opposed to ROS poses :s)
                    try
                    {
                        geometry_msgs::msg::Vector3Stamped downwind_map;
                        {
                            downwind_map.header.frame_id = input_fixed_frame;
                            downwind_map.header.stamp = now();
                            downwind_map.vector.x = u;
                            downwind_map.vector.y = v;
                            downwind_map.vector.z = w;
                        }
                        geometry_msgs::msg::Vector3Stamped downwind_sensor;

                        tf2::doTransform(downwind_map, downwind_sensor, anemometer_transform_map);
                        wind_direction = std::atan2(-downwind_sensor.vector.y, -downwind_sensor.vector.x); // change signs to make it upwind
                    }
                    catch (tf2::TransformException& ex)
                    {
                        RCLCPP_ERROR(get_logger(), "%s - Error: %s", __FUNCTION__, ex.what());
                    }
                }
                else
                {
                    // for simulations
                    wind_direction = std::atan2(v, u);
                }

                // Adding Noise
                static RandomGenerator rng(static_cast<unsigned>(time(0)));
                static NormalDistribution gaussian_dist(0.0, noise_std);
                wind_direction = wind_direction + gaussian_dist(rng);
                wind_speed = std::max(0.0, wind_speed + gaussian_dist(rng) * 0.1f);

                // Publish 2D Anemometer readings
                //------------------------------
                anemo_msg.header.stamp = now();
                if (use_map_ref_system)
                    anemo_msg.header.frame_id = input_fixed_frame;
                else
                    anemo_msg.header.frame_id = input_sensor_frame;

                anemo_msg.sensor_label = "Fake_Anemo";
                anemo_msg.wind_direction = wind_direction; // rad
                anemo_msg.wind_speed = wind_speed;         // m/s
                // Publish fake_anemometer reading (m/s)
                sensor_read_pub->publish(anemo_msg);

                // Add wind marker ARROW for Rviz (2D) --> Upwind
                /*
                wind_point.header.stamp = now();
                wind_point.points.clear();
                wind_point.id = 1;  //unique identifier for each arrow
                wind_point.pose.position.x = 0.0;
                wind_point.pose.position.y = 0.0;
                wind_point.pose.position.z = 0.0;
                wind_point.pose.orientation = tf2_ros::createQuaternionMsgFromYaw(wind_direction_with_noise);
                wind_point.scale.x = 2*sqrt(pow(u,2)+pow(v,2));	  //arrow lenght
                wind_point.scale.y = 0.1;	  //arrow width
                wind_point.scale.z = 0.1;	  //arrow height
                wind_point.color.r = 0.0;
                wind_point.color.g = 1.0;
                wind_point.color.b = 0.0;
                wind_point.color.a = 1.0;
                marker_pub.publish(wind_point);
                */

                // Add inverted wind marker --> DownWind
                wind_point_inv.header.stamp = now();
                wind_point_inv.header.frame_id = anemo_msg.header.frame_id;
                wind_point_inv.points.clear();
                wind_point_inv.id = 1; // unique identifier for each arrow
                if (use_map_ref_system)
                {
                    wind_point_inv.pose.position.x = anemometer_transform_map.transform.translation.x;
                    wind_point_inv.pose.position.y = anemometer_transform_map.transform.translation.y;
                    wind_point_inv.pose.position.z = anemometer_transform_map.transform.translation.z;
                }
                else
                {
                    wind_point_inv.pose.position.x = 0.0;
                    wind_point_inv.pose.position.y = 0.0;
                    wind_point_inv.pose.position.z = 0.0;
                }

                tf2::Quaternion arrowOrientation(tf2::Vector3(0, 0, 1), wind_direction + M_PI);
                wind_point_inv.pose.orientation = tf2::toMsg(arrowOrientation);
                wind_point_inv.scale.x = 2 * std::sqrt(u * u + v * v); // arrow lenght
                wind_point_inv.scale.y = 0.1;                          // arrow width
                wind_point_inv.scale.z = 0.1;                          // arrow height
                wind_point_inv.color.r = 0.0;
                wind_point_inv.color.g = 1.0;
                wind_point_inv.color.b = 0.0;
                wind_point_inv.color.a = 1.0;
                marker_pub->publish(wind_point_inv);
            }
            else
            {
                RCLCPP_WARN(get_logger(), "[fake_anemometer] Cannot read Wind Vector from simulated data.");
            }

            // Publish RVIZ sensor pose (a sphere)
            /*
            sensor.header.stamp = now();
            sensor.pose.position.x = x_pos;
            sensor.pose.position.y = y_pos;
            sensor.pose.position.z = z_pos;
            marker_pub.publish(sensor);
            */

            // PUBLISH ANEMOMETER Stick
            /*
            connector.header.stamp = now();
            connector.scale.z = z_pos;
            connector.pose.position.x = x_pos;
            connector.pose.position.y = y_pos;
            connector.pose.position.z = float(z_pos)/2;
            marker_pub.publish(connector);
            */
        }
        r.sleep();
    }
}

// Load Sensor parameters
void SimulatedAnemometer::loadNodeParameters()
{
    // sensor_frame
    input_sensor_frame = declare_parameter<std::string>("sensor_frame", "anemometer_link");

    // fixed frame
    input_fixed_frame = declare_parameter<std::string>("fixed_frame", "map");

    // Noise
    noise_std = declare_parameter<double>("noise_std", 0.1);

    // Frequency (Hz)
    frequency = declare_parameter<double>("frequency", 20);

    // What ref system to use for publishing measurements
    use_map_ref_system = declare_parameter<bool>("use_map_ref_system", false);

    RCLCPP_INFO(get_logger(), "wind noise: %f", noise_std);
}
