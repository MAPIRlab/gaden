#pragma once
#include <rclcpp/rclcpp.hpp>


#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>


#include <gaden_environment/srv/occupancy.hpp>
#include <gaden_player/srv/gas_position.hpp>
#include <olfaction_msgs/msg/gas_sensor.hpp>
#include "glm/glm.hpp"

#include <DDA.h>
#include <vector>

class TDLAS : public rclcpp::Node
{
public:
    TDLAS();
    ~TDLAS() = default;

    void run();

private:
    rclcpp::Publisher<olfaction_msgs::msg::GasSensor>::SharedPtr m_readingsPub{nullptr};
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_markerPub{nullptr};
    rclcpp::Client<gaden_player::srv::GasPosition>::SharedPtr m_playerClient{nullptr};
    rclcpp::Subscription<gaden_environment::srv::Occupancy>::SharedPtr m_mapSubscriber{nullptr};

    bool m_verbose;
    std::string m_fixedFrame;
    std::string m_sensor_frame;
    
    float m_maxRayDistance;
    float m_rayMarchResolution; 
    float m_measurementFrequency;
    std::vector<std::vector<std::vector<bool>> > m_map;
    glm::vec3 m_mapOrigin;

    void getEnvironment();
    double takeMeasurement();

    struct PositionAndDirection
    {
        geometry_msgs::msg::Transform pose;
        tf2::Vector3 forward()
        { 
            tf2::Quaternion quat;
            tf2::fromMsg(pose.rotation, quat);
            return tf2::quatRotate(quat, tf2::Vector3{1,0,0} ); 
        }

        static geometry_msgs::msg::Point vec_to_point(const geometry_msgs::msg::Vector3& vec)
        {
            geometry_msgs::msg::Point p;
            p.x = vec.x;
            p.y = vec.y;
            p.z = vec.z;
            
            return p;
        }
        static geometry_msgs::msg::Point vec_to_point(const tf2::Vector3& vec)
        {
            geometry_msgs::msg::Point p;
            p.x = vec.x();
            p.y = vec.y();
            p.z = vec.z();
            
            return p;
        }
    };
    PositionAndDirection m_poseInFixedFrame;
    void updatePoseInFixedFrame();

    glm::vec3 m_endPointLastMeasurement;
    void publish(double measured);
};