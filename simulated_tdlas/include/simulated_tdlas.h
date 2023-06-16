#pragma once
#include <rclcpp/rclcpp.hpp>


#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>


#include <gaden_environment/srv/occupancy.hpp>
#include <gaden_player/srv/gas_position.hpp>
#include <olfaction_msgs/msg/tdlas.hpp>

#include <DDA/DDA.h>
#include <vector>

#include <gaden_common/Vector_conversions.h>

class TDLAS : public rclcpp::Node
{
public:
    TDLAS();
    ~TDLAS() = default;

    void run();

private:

    std::unique_ptr<tf2_ros::Buffer> m_tfBuffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tfListener;

    rclcpp::Publisher<olfaction_msgs::msg::TDLAS>::SharedPtr m_readingsPub{nullptr};
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
    Gaden::Vector3 m_mapOrigin;

    void getEnvironment();
    double takeMeasurement();

    struct PositionAndDirection
    {
        geometry_msgs::msg::Transform pose;
        Gaden::Vector3 forward()
        { 
            tf2::Quaternion quat;
            tf2::fromMsg(pose.rotation, quat);
            return Gaden::fromTF( tf2::quatRotate(quat, tf2::Vector3{1,0,0} ) ); 
        }
    };
    PositionAndDirection m_poseInFixedFrame;
    void updatePoseInFixedFrame();

    Gaden::Vector3 m_endPointLastMeasurement;
    void publish(double measured);
    

    // optional reflector data. If you are using a second robot to reflect the laser off of
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr m_reflectorLocSub;
    void reflectorLocCB(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    struct Cylinder
    {
        glm::vec3 baseCenter;
        float radius = 0;
        float height = 0;
    };
    Cylinder m_reflectorRobot;
};