#include "simulated_tdlas.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tdlas");
    TDLAS tdlas;
    tdlas.run();
    
    return 0;
}

TDLAS::TDLAS()
{
    ros::NodeHandle privateNH("~");

    //read params from launchfile
    m_measurementFrequency = privateNH.param<float>("measurementFrequency", 2);
    m_rayMarchResolution = privateNH.param<float>("rayMarchResolution", 0.1);
    m_maxRayDistance = privateNH.param<float>("maxRayDistance", 10);

    m_verbose = privateNH.param<bool>("verbose", false);
    
    m_fixedFrame = privateNH.param<std::string>("fixedFrame", "map");
    m_sensor_frame = privateNH.param<std::string>("sensorFrame", "tdlas_frame");

    m_readingsPub = m_nodeHandle.advertise<olfaction_msgs::gas_sensor>("/tdlas/reading", 100);
    m_markerPub = m_nodeHandle.advertise<visualization_msgs::Marker>("/tdlas/arrow", 100);
    m_playerClient = m_nodeHandle.serviceClient<gaden_player::GasPosition>("/odor_value");
}

void TDLAS::run()
{

    getEnvironment();

    ros::Rate rate(m_measurementFrequency);
    while(ros::ok())
    {
        ros::spinOnce();

        updatePoseInFixedFrame();
        double measured = takeMeasurement();        
        publish(measured);

        rate.sleep();
    }
}

void TDLAS::getEnvironment()
{
    ros::ServiceClient client = m_nodeHandle.serviceClient<gaden_environment::Occupancy>("gaden_environment/occupancyMap3D");

    gaden_environment::OccupancyRequest request;
    gaden_environment::OccupancyResponse response;
    
    ros::Rate r(1);
    while(!client.call(request, response))
    {
        ROS_INFO("[TDLAS] WAITING FOR GADEN_ENVIRONMENT/OCCUPANCY SERVICE");
        r.sleep();
    }

    double resoultionRatio = response.resolution / m_rayMarchResolution;
    int numCellsX = response.numCellsX * resoultionRatio;
    int numCellsY = response.numCellsY * resoultionRatio;
    int numCellsZ = response.numCellsZ * resoultionRatio;
    
    m_mapOrigin.x = response.origin.x;
    m_mapOrigin.y = response.origin.y;
    m_mapOrigin.z = response.origin.z;
    
    m_map.resize(numCellsX, std::vector<std::vector<bool>>(numCellsY, std::vector<bool>(numCellsZ) ) );


    auto cellFree = [&response, resoultionRatio](int argI, int argJ, int argK)
    {
        int nx = response.numCellsX;
        int ny = response.numCellsY;
        bool cellIsFree = true;
        for(int i = argI/resoultionRatio; i<(argI+1)/resoultionRatio; i++)
        {
            for(int j = argJ/resoultionRatio; j<(argJ+1)/resoultionRatio; j++)
            {
                for(int k = argK/resoultionRatio; k<(argK+1)/resoultionRatio; k++)
                {
                    int value = response.occupancy[i + j*nx + k*nx*ny];
                    cellIsFree = cellIsFree &&  value == 0;
                }
            }
        }
        return cellIsFree;
    };

    for(int i=0; i<m_map.size(); i++)
    {
        for(int j=0; j<m_map[0].size(); j++)
        {
            for(int k=0; k<m_map[0][0].size(); k++)
            {
                m_map[i][j][k] = cellFree(i,j,k);
            }
        }
    }
    
}

void TDLAS::updatePoseInFixedFrame()
{
    static tf2_ros::Buffer buffer;
    static tf2_ros::TransformListener listener(buffer);

    geometry_msgs::PointStamped locationInSensorFrame;
    tf::pointStampedTFToMsg( tf::Stamped<tf::Point>({0,0,0}, ros::Time(0), m_sensor_frame), locationInSensorFrame);
    
    geometry_msgs::Vector3Stamped directionInSensorFrame;
    tf::vector3StampedTFToMsg( tf::Stamped<tf::Vector3>({1,0,0}, ros::Time(0), m_sensor_frame), directionInSensorFrame );

    try{
        geometry_msgs::TransformStamped transform = buffer.lookupTransform(m_fixedFrame, m_sensor_frame, ros::Time(0));
        m_poseInFixedFrame.position = buffer.transform(locationInSensorFrame, m_fixedFrame);
        m_poseInFixedFrame.forward = buffer.transform(directionInSensorFrame, m_fixedFrame);
    }
    catch(const std::exception& e)
    {
        ROS_ERROR("Exception transforming from sensor frame to map frame: %s", e.what());
    }
}

double TDLAS::takeMeasurement()
{
    //run the DDA algorithm
    glm::vec3 rayOrigin;
    rayOrigin.x = m_poseInFixedFrame.position.point.x;
    rayOrigin.y = m_poseInFixedFrame.position.point.y;
    rayOrigin.z = m_poseInFixedFrame.position.point.z;

    glm::vec3 rayDirection;
    rayDirection.x = m_poseInFixedFrame.forward.vector.x;
    rayDirection.y = m_poseInFixedFrame.forward.vector.y;
    rayDirection.z = m_poseInFixedFrame.forward.vector.z;

    static std::function<bool(bool)> identity = [](const bool& b){return b;};
    DDA::_3D::RayMarchInfo rayData =  DDA::_3D::marchRay<bool>(rayOrigin, rayDirection, m_maxRayDistance, 
        m_map, identity, 
        m_mapOrigin, m_rayMarchResolution);

    //record endpoint for the rviz marker
    if(rayData.lengthInCell.size()!=0)
        m_endPointLastMeasurement = rayOrigin + glm::normalize(rayDirection) * rayData.totalLength;
    else
        m_endPointLastMeasurement = rayOrigin;



    // Actually get the measurement
    gaden_player::GasPosition srv;
    for(const auto& pair : rayData.lengthInCell)
    {
        glm::vec3 coords = glm::vec3(pair.first) * m_rayMarchResolution + m_mapOrigin;
        srv.request.x.push_back(coords.x);
        srv.request.y.push_back(coords.y);
        srv.request.z.push_back(coords.z);
    }

    double totalMeasured = 0;
    if(m_playerClient.call(srv))
    {
        for( int i=0; i<srv.response.positions.size(); i++)
        {
            for(int g=0; g<srv.response.gas_type.size(); g++)
            {
                //TODO add a hashmap that maps gas type to strength of sensor response for other gases
                if(srv.response.gas_type[g] == "methane")
                {
                    totalMeasured += srv.response.positions[i].concentration[g] * rayData.lengthInCell[i].second;
                }
            }
        }
    }
    else
        ROS_ERROR("[TDLAS] Can't read concentrations from gaden_player");

    if(m_verbose)
        ROS_INFO("[TDLAS] %f ppm x m", totalMeasured);

    return totalMeasured;
}

void TDLAS::publish(double measured)
{
    {
        olfaction_msgs::gas_sensor msg;
    
        msg.header.frame_id = m_sensor_frame;
        msg.header.stamp = ros::Time::now();

        msg.technology = msg.TECH_TDLAS;
        msg.mpn = msg.MPN_NOT_VALID;
        msg.manufacturer = msg.MANU_UNKNOWN;
        
        msg.raw = measured;
        msg.raw_units = msg.UNITS_PPMxM;
        msg.raw_air = 0;
        
        m_readingsPub.publish(msg);
    }

    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = m_fixedFrame;
        marker.header.stamp = ros::Time::now();
        marker.type = marker.ARROW;
        marker.scale.x = 0.1;
        marker.scale.y = 0.2;
        marker.scale.z = 1;
        
        marker.color.r = 1;
        marker.color.a = 1;

        marker.points.push_back(m_poseInFixedFrame.position.point);

        geometry_msgs::Point endPoint;
        tf::pointTFToMsg(tf::Point(
            m_endPointLastMeasurement.x, 
            m_endPointLastMeasurement.y,
            m_endPointLastMeasurement.z
        ), endPoint);
        marker.points.push_back(endPoint);

        m_markerPub.publish(marker);
    }
}


