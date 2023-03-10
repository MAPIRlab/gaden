#pragma once
#include <ros/ros.h>
#include <vector>


#include <tf/tf.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <visualization_msgs/Marker.h>

#include <gaden_environment/Occupancy.h>
#include <gaden_player/GasPosition.h>
#include <olfaction_msgs/gas_sensor.h>
#include "glm/glm.hpp"

#include "DDA.h"

class TDLAS
{
public:
    TDLAS();
    ~TDLAS() = default;

    void run();

private:
    ros::NodeHandle m_nodeHandle;
    ros::Publisher m_readingsPub;
    ros::Publisher m_markerPub;
    ros::ServiceClient m_playerClient;
    ros::Subscriber m_mapSubscriber;

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
        geometry_msgs::PointStamped position;
        geometry_msgs::Vector3Stamped forward;
    };
    PositionAndDirection m_poseInFixedFrame;
    void updatePoseInFixedFrame();

    glm::vec3 m_endPointLastMeasurement;
    void publish(double measured);
};