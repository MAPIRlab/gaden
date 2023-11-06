#ifdef GENERATE_COPPELIA_SCENE
#define SIM_REMOTEAPICLIENT_OBJECTS
#include <RemoteAPIClient.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <fmt/format.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
#ifndef GENERATE_COPPELIA_SCENE
    RCLCPP_ERROR(rclcpp::get_logger("coppelia"),
                 "You did not compile preprocessing with coppelia support! There is an option to change this in the CMakeLists.txt");
    return -1;

#else

    // Params
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("MoveCoppeliaObject");
    std::string robotName = node->declare_parameter<std::string>("robotName", "PioneerP3DX");
    bool permanentChange = node->declare_parameter<bool>("permanentChange", false);
    float simulationSpeed = node->declare_parameter<float>("simulationSpeed", 1.0);
    std::string setPoseTopic = node->declare_parameter<std::string>("setPoseTopic", "initialpose");

#define POSITION_NOT_SET DBL_MAX
    std::vector<double> positionToSet =
        node->declare_parameter<std::vector<double>>("position", {POSITION_NOT_SET, POSITION_NOT_SET, POSITION_NOT_SET});
    if (positionToSet[0] == POSITION_NOT_SET)
    {
        RCLCPP_ERROR(rclcpp::get_logger("coppelia"), "Cannot run this node without setting the \"position\" parameter.");
        return -1;
    }

    // Let's go
    RemoteAPIClient client;
    RemoteAPIObject::sim sim = client.getObject().sim();

    sim.stopSimulation();
    while (sim.getSimulationState() != sim.simulation_stopped)
        ;

    int64_t robot_root_handle = sim.getObject(fmt::format("/MobileRobots/{}_root", robotName));
    sim.setObjectPosition(robot_root_handle, sim.handle_world, positionToSet);
    positionToSet[0] = POSITION_NOT_SET;

    // probably unnecessary, but might as well
    sim.announceSceneContentChange();

    if (permanentChange)
        sim.saveScene(sim.getStringParam(sim.stringparam_scene_path_and_name)); // overwrite existing scene

    using Pose = geometry_msgs::msg::PoseWithCovarianceStamped;
    rclcpp::Subscription<Pose>::SharedPtr setPoseSub =
        node->create_subscription<Pose>(setPoseTopic, 1,
                                        [&positionToSet](Pose::SharedPtr pose) {
                                            positionToSet = {pose->pose.pose.position.x, pose->pose.pose.position.y, pose->pose.pose.position.z};
                                        });

    int64_t robot_handle = sim.getObjectChild(robot_root_handle, 0);

    sim.startSimulation();
    client.setStepping(true);

    double period = sim.getFloatParam(sim.floatparam_simulation_time_step) / simulationSpeed;
    rclcpp::Rate rate(1.0 / period);
    while (rclcpp::ok())
    {
        client.step();
        rate.sleep();
        rclcpp::spin_some(node);
        if (positionToSet[0] != POSITION_NOT_SET)
        {

            for (int64_t handle : sim.getObjectsInTree(robot_handle))
                sim.resetDynamicObject(handle);

            RCLCPP_WARN(rclcpp::get_logger("coppelia"), "Setting position (%.2f, %.2f, %.2f)", positionToSet[0], positionToSet[1], positionToSet[2]);

            positionToSet[2] = sim.getObjectPosition(robot_handle, sim.handle_world)[2];
            sim.setObjectPosition(robot_handle, sim.handle_world, positionToSet);
            while (sim.getObjectPosition(robot_handle, sim.handle_world)[0] != positionToSet[0])
                ;
            positionToSet[0] = POSITION_NOT_SET;
        }
    }
#endif
}