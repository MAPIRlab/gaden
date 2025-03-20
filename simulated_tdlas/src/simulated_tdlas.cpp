#include "simulated_tdlas.h"
#include <tf2_ros/buffer_interface.h>
using namespace std::chrono_literals;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<TDLAS> tdlas = std::make_shared<TDLAS>();
    tdlas->run();

    return 0;
}

TDLAS::TDLAS() : rclcpp::Node("Simulated_tdlas")
{
    m_tfBuffer = std::make_unique<tf2_ros::Buffer>(get_clock());
    m_tfListener = std::make_unique<tf2_ros::TransformListener>(*m_tfBuffer);

    // read params from launchfile
    m_measurementFrequency = declare_parameter<float>("measurementFrequency", 2);
    m_rayMarchResolution = declare_parameter<float>("rayMarchResolution", 0.1);
    m_maxRayDistance = declare_parameter<float>("maxRayDistance", 10);

    m_verbose = declare_parameter<bool>("verbose", false);

    m_fixedFrame = declare_parameter<std::string>("fixedFrame", "map");
    m_sensor_frame = declare_parameter<std::string>("sensorFrame", "tdlas_frame");

    m_readingsPub = create_publisher<olfaction_msgs::msg::TDLAS>("tdlas/reading", 100);
    m_markerPub = create_publisher<visualization_msgs::msg::Marker>("tdlas/arrow", 100);
    m_playerClient = create_client<gaden_msgs::srv::GasPosition>("/odor_value");

    std::string reflectorLocTopic = declare_parameter<std::string>("reflectorLocTopic", "/reflector/amcl_pose");
    m_reflectorLocSub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        reflectorLocTopic, 1, std::bind(&TDLAS::reflectorLocCB, this, std::placeholders::_1));

    m_reflectorRobot.radius = declare_parameter<float>("reflector_radius", 0.3);
    m_reflectorRobot.height = declare_parameter<float>("reflector_height", 2);
}

void TDLAS::run()
{
    auto shared_this = shared_from_this();
    while (rclcpp::ok() && !m_playerClient->wait_for_service(5s))
        RCLCPP_INFO(get_logger(), "WAITING FOR GADEN_PLAYER SERVICE");

    getEnvironment();

    rclcpp::Rate rate(m_measurementFrequency);
    while (rclcpp::ok())
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

    gaden_msgs::srv::Occupancy::Response::SharedPtr response{nullptr};
    {
        auto client = create_client<gaden_msgs::srv::Occupancy>("/gaden_environment/occupancyMap3D");
        while (rclcpp::ok() && !client->wait_for_service(5s))
            RCLCPP_INFO(get_logger(), "WAITING FOR GADEN_ENVIRONMENT/OCCUPANCY SERVICE");

        auto request = std::make_shared<gaden_msgs::srv::Occupancy::Request>();
        rclcpp::Rate wait_rate(1);
        bool done = false;
        while (!done)
        {
            auto result = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(shared_from_this(), result, 10s) == rclcpp::FutureReturnCode::SUCCESS)
            {
                response = result.get();
                done = true;
            }
            else
            {
                RCLCPP_ERROR(get_logger(), "DID NOT GET AN ANSWER FROM GADEN_ENVIRONMENT/OCCUPANCY SERVICE");
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

    m_map.resize(num_cells_x, std::vector<std::vector<bool>>(num_cells_y, std::vector<bool>(num_cells_z)));

    auto cellFree = [&response, resoultionRatio](int argI, int argJ, int argK)
    {
        int nx = response->num_cells_x;
        int ny = response->num_cells_y;
        bool cellIsFree = true;
        for (int i = argI / resoultionRatio; i < (argI + 1) / resoultionRatio; i++)
        {
            for (int j = argJ / resoultionRatio; j < (argJ + 1) / resoultionRatio; j++)
            {
                for (int k = argK / resoultionRatio; k < (argK + 1) / resoultionRatio; k++)
                {
                    int value = response->occupancy[i + j * nx + k * nx * ny];
                    cellIsFree = cellIsFree && value == 0;
                }
            }
        }
        return cellIsFree;
    };

    for (int i = 0; i < m_map.size(); i++)
    {
        for (int j = 0; j < m_map[0].size(); j++)
        {
            for (int k = 0; k < m_map[0][0].size(); k++)
            {
                m_map[i][j][k] = cellFree(i, j, k);
            }
        }
    }
}

void TDLAS::updatePoseInFixedFrame()
{

    tf2::Stamped<tf2::Transform> poseInSensorFrame(tf2::Transform(tf2::Quaternion::getIdentity(), {0, 0, 0}), tf2_ros::fromRclcpp(now()),
                                                   m_sensor_frame);
    geometry_msgs::msg::TransformStamped poseInSensorFrame_msg = tf2::toMsg(poseInSensorFrame);

    try
    {
        geometry_msgs::msg::TransformStamped poseInFixedFrame_msg = m_tfBuffer->transform(poseInSensorFrame_msg, m_fixedFrame);
        m_poseInFixedFrame.pose = poseInFixedFrame_msg.transform;
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(get_logger(), "Exception transforming from sensor frame to map frame: %s", e.what());
    }
}

double TDLAS::takeMeasurement()
{
    // run the DDA algorithm
    gaden::Vector3 rayOrigin = gaden::fromGeoMSg(m_poseInFixedFrame.pose.translation);

    gaden::Vector3 rayDirection = m_poseInFixedFrame.forward();

    static auto identity = [](const bool& b) { return b; };

    static auto doesNotCollideWithReflector = [this](const glm::vec3& position)
    {
        if (position.z < m_reflectorRobot.baseCenter.z || position.z > m_reflectorRobot.baseCenter.z + m_reflectorRobot.height)
            return true;
        glm::vec3 projectedCenter(m_reflectorRobot.baseCenter.x, m_reflectorRobot.baseCenter.y, position.z);
        return glm::distance(position, projectedCenter) > m_reflectorRobot.radius;
    };

    static DDA::_3D::Map<bool> DDAMap {m_map, m_mapOrigin, m_rayMarchResolution};
    DDA::_3D::RayMarchInfo rayData = DDA::_3D::marchRay<bool>(rayOrigin, rayDirection, m_maxRayDistance, DDAMap,
                                                              identity, doesNotCollideWithReflector);

    // record endpoint for the rviz marker
    if (rayData.lengthInCell.size() != 0)
        m_endPointLastMeasurement = rayOrigin + glm::normalize(rayDirection) * rayData.totalLength;
    else
        m_endPointLastMeasurement = rayOrigin;

    // Actually get the measurement
    auto request = std::make_shared<gaden_msgs::srv::GasPosition::Request>();
    for (const auto& pair : rayData.lengthInCell)
    {
        gaden::Vector3 coords = gaden::Vector3(pair.first) * m_rayMarchResolution + m_mapOrigin;
        request->x.push_back(coords.x);
        request->y.push_back(coords.y);
        request->z.push_back(coords.z);
    }

    double totalMeasured = 0;
    auto future = m_playerClient->async_send_request(request);
    if (rclcpp::spin_until_future_complete(shared_from_this(), future, 1s) == rclcpp::FutureReturnCode::SUCCESS)
    {
        auto response = future.get();
        for (int i = 0; i < response->positions.size(); i++)
        {
            for (int g = 0; g < response->gas_type.size(); g++)
            {
                // TODO add a hashmap that maps gas type to strength of sensor response for other gases
                if (response->gas_type[g] == "methane")
                {
                    totalMeasured += response->positions[i].concentration[g] * rayData.lengthInCell[i].second;
                }
            }
        }
    }
    else
        RCLCPP_ERROR(get_logger(), "[TDLAS] Can't read concentrations from gaden_player");

    if (m_verbose)
        RCLCPP_INFO(get_logger(), "[TDLAS] %f ppm x m", totalMeasured);

    return totalMeasured;
}

void TDLAS::publish(double measured)
{
    {
        olfaction_msgs::msg::TDLAS msg;

        msg.header.frame_id = m_sensor_frame;
        msg.header.stamp = now();

        msg.average_ppmxm = measured;
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

        marker.points.push_back(gaden::geoMsgToPoint(m_poseInFixedFrame.pose.translation));

        geometry_msgs::msg::Point endPoint = gaden::toPoint(m_endPointLastMeasurement);

        marker.points.push_back(endPoint);

        m_markerPub->publish(marker);
    }
}

void TDLAS::reflectorLocCB(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    geometry_msgs::msg::PoseStamped pose_original_frame;
    pose_original_frame.header = msg->header;
    pose_original_frame.pose = msg->pose.pose;

    geometry_msgs::msg::PoseStamped pose_fixed_frame = m_tfBuffer->transform(pose_original_frame, m_fixedFrame);
    m_reflectorRobot.baseCenter = gaden::fromPoint(pose_fixed_frame.pose.position);
}
