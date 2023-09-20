#ifdef GENERATE_COPPELIA_SCENE
#define SIM_REMOTEAPICLIENT_OBJECTS
#include <RemoteAPIClient.h>
#endif

#include <rclcpp/rclcpp.hpp>
#include <gaden_common/Utils.h>
#include <fmt/format.h>


int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	#ifndef GENERATE_COPPELIA_SCENE
	RCLCPP_ERROR(rclcpp::get_logger("coppelia"), "You did not compile preprocessing with coppelia support! There is an option to change this in the CMakeLists.txt");
	return -1;

	#else

	// Params
	rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("MoveCoppeliaObject");
	bool permanentChange = node->declare_parameter<bool>("permanentChange", false);

	#define POSITION_NOT_SET DBL_MAX
	std::vector<double> position = node->declare_parameter<std::vector<double>>("position", { POSITION_NOT_SET, POSITION_NOT_SET, POSITION_NOT_SET });
	if (position[0] == POSITION_NOT_SET)
	{
		RCLCPP_ERROR(rclcpp::get_logger("coppelia"), "Cannot run this node without setting the \"position\" parameter.");
		return -1;
	}

	std::string robotName = node->declare_parameter<std::string>("robotName", "PioneerP3DX");


	//Let's go
	RemoteAPIClient client;
	RemoteAPIObject::sim sim = client.getObject().sim();

	sim.stopSimulation();
	int64_t robot_handle = sim.getObject(fmt::format("/MobileRobots/{}_root", robotName));
	sim.setObjectPosition(robot_handle, sim.handle_world, position);

	auto BB = sim.getShapeBB(robot_handle);
	RCLCPP_INFO(rclcpp::get_logger("coppelia"), "BB: %f, %f, %f", BB[0], BB[1], BB[2]);

	//probably unnecessary, but might as well
	sim.resetDynamicObject(robot_handle);
	sim.announceSceneContentChange();

	if (permanentChange)
		sim.saveScene(sim.getStringParam(sim.stringparam_scene_path_and_name)); //overwrite existing scene

	sim.startSimulation();
	#endif
}