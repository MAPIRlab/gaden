#include "simulated_tdlas.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<TDLAS> tdlas = std::make_shared<TDLAS>();
    tdlas->run();
    
    return 0;
}

TDLAS::TDLAS() : rclcpp::Node("Simulated_tdlas")
{
    //read params from launchfile
    m_measurementFrequency = declare_parameter<float>("measurementFrequency", 2);
    m_rayMarchResolution = declare_parameter<float>("rayMarchResolution", 0.1);
    m_maxRayDistance = declare_parameter<float>("maxRayDistance", 10);

    m_verbose = declare_parameter<bool>("verbose", false);
    
    m_fixedFrame = declare_parameter<std::string>("fixedFrame", "map");
    m_sensor_frame = declare_parameter<std::string>("sensorFrame", "tdlas_frame");

    m_readingsPub = create_publisher<olfaction_msgs::msg::GasSensor>("/tdlas/reading", 100);
    m_markerPub = create_publisher<visualization_msgs::msg::Marker>("/tdlas/arrow", 100);
    m_playerClient = create_client<gaden_player::srv::GasPosition>("/odor_value");
}

void TDLAS::run()
{
    auto shared_this = shared_from_this();
    getEnvironment();

    rclcpp::Rate rate(m_measurementFrequency);
    while(rclcpp::ok())
    {
        rclcpp::spin_some(shared_this);

        updatePoseInFixedFrame();
        double measured = takeMeasurement();        
        publish(measured);

        rate.sleep();
    }
}

void TDLAS::getEnvironment()
{

    gaden_environment::srv::Occupancy::Response::SharedPtr response{nullptr};
    {
        auto client = create_client<gaden_environment::srv::Occupancy>("gaden_environment/occupancyMap3D");
        auto request = std::make_shared<gaden_environment::srv::Occupancy::Request>();
        rclcpp::Rate wait_rate(1);
        bool done = false;
        while(!done)
        {
            auto result = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(shared_from_this(), result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                response = result.get();
                done = true;
            }
            else
            {
                RCLCPP_INFO(get_logger(), "WAITING FOR GADEN_ENVIRONMENT/OCCUPANCY SERVICE");
                wait_rate.sleep();
            }
        }
    }

    double resoultionRatio = response->resolution / m_rayMarchResolution;
    int num_cells_x = response->num_cells_x * resoultionRatio;
    int num_cells_y = response->num_cells_y * resoultionRatio;
    int num_cells_z = response->num_cells_z * resoultionRatio;
    
    m_mapOrigin.x = response->origin.x;
    m_mapOrigin.y = response->origin.y;
    m_mapOrigin.z = response->origin.z;
    
    m_map.resize(num_cells_x, std::vector<std::vector<bool>>(num_cells_y, std::vector<bool>(num_cells_z) ) );


    auto cellFree = [&response, resoultionRatio](int argI, int argJ, int argK)
    {
        int nx = response->num_cells_x;
        int ny = response->num_cells_y;
        bool cellIsFree = true;
        for(int i = argI/resoultionRatio; i<(argI+1)/resoultionRatio; i++)
        {
            for(int j = argJ/resoultionRatio; j<(argJ+1)/resoultionRatio; j++)
            {
                for(int k = argK/resoultionRatio; k<(argK+1)/resoultionRatio; k++)
                {
                    int value = response->occupancy[i + j*nx + k*nx*ny];
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
    static tf2_ros::Buffer buffer(get_clock());
    static tf2_ros::TransformListener listener(buffer);

    tf2::Stamped<tf2::Transform> poseInSensorFrame(
        tf2::Transform(tf2::Quaternion::getIdentity(), {0,0,0}),
        tf2::get_now(),
        m_sensor_frame
    );
    geometry_msgs::msg::TransformStamped poseInSensorFrame_msg = tf2::toMsg(poseInSensorFrame);

    try{
        geometry_msgs::msg::TransformStamped poseInFixedFrame_msg = buffer.transform(poseInSensorFrame_msg, m_fixedFrame);
        m_poseInFixedFrame.pose = poseInFixedFrame_msg.transform;
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "Exception transforming from sensor frame to map frame: %s", e.what());
    }
}

double TDLAS::takeMeasurement()
{
    //run the DDA algorithm
    glm::vec3 rayOrigin;
    rayOrigin.x = m_poseInFixedFrame.pose.translation.x;
    rayOrigin.y = m_poseInFixedFrame.pose.translation.y;
    rayOrigin.z = m_poseInFixedFrame.pose.translation.z;

    glm::vec3 rayDirection;
    rayDirection.x = m_poseInFixedFrame.forward().x();
    rayDirection.y = m_poseInFixedFrame.forward().y();
    rayDirection.z = m_poseInFixedFrame.forward().z();

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
    auto request = std::make_shared<gaden_player::srv::GasPosition::Request>();
    for(const auto& pair : rayData.lengthInCell)
    {
        glm::vec3 coords = glm::vec3(pair.first) * m_rayMarchResolution + m_mapOrigin;
        request->x.push_back(coords.x);
        request->y.push_back(coords.y);
        request->z.push_back(coords.z);
    }

    double totalMeasured = 0;
    auto future = m_playerClient->async_send_request(request);
    if(rclcpp::spin_until_future_complete(shared_from_this(), future) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        for( int i=0; i<response->positions.size(); i++)
        {
            for(int g=0; g<response->gas_type.size(); g++)
            {
                //TODO add a hashmap that maps gas type to strength of sensor response for other gases
                if(response->gas_type[g] == "methane")
                {
                    totalMeasured += response->positions[i].concentration[g] * rayData.lengthInCell[i].second;
                }
            }
        }
    }
    else
        RCLCPP_ERROR(get_logger(), "[TDLAS] Can't read concentrations from gaden_player");

    if(m_verbose)
        RCLCPP_INFO(get_logger(), "[TDLAS] %f ppm x m", totalMeasured);

    return totalMeasured;
}

void TDLAS::publish(double measured)
{
    {
        olfaction_msgs::msg::GasSensor msg;
    
        msg.header.frame_id = m_sensor_frame;
        msg.header.stamp = now();

        msg.technology = msg.TECH_TDLAS;
        msg.mpn = msg.MPN_NOT_VALID;
        msg.manufacturer = msg.MANU_UNKNOWN;
        
        msg.raw = measured;
        msg.raw_units = msg.UNITS_PPMXM;
        msg.raw_air = 0;
        
        m_readingsPub->publish(msg);
    }

    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = m_fixedFrame;
        marker.header.stamp = now();
        marker.type = marker.ARROW;
        marker.scale.x = 0.1;
        marker.scale.y = 0.2;
        marker.scale.z = 1;
        
        marker.color.r = 1;
        marker.color.a = 1;

        marker.points.push_back( PositionAndDirection::vec_to_point( m_poseInFixedFrame.pose.translation) );

        geometry_msgs::msg::Point endPoint = PositionAndDirection::vec_to_point(
            tf2::Vector3(
                m_endPointLastMeasurement.x, 
                m_endPointLastMeasurement.y,
                m_endPointLastMeasurement.z
            )
        );

        marker.points.push_back(endPoint);

        m_markerPub->publish(marker);
    }
}


