/*
 * The only goal of this Node is to display the simulation environment and gas source location in RVIZ.
 *
 * 1. Loads the simulation environment (usually from CFD in the file format .env), and displays it as RVIZ markers.
 * 2. Displays the Gas-Source Location as two cylinders.
 */

#include "environment/environment.h"

 // ===============================//
 //              MAIN              //
 // ===============================//
int main(int argc, char** argv)
{
	//Init
	ros::init(argc, argv, "environment");

	Environment environment;
	//Load Parameters
	environment.run();

}

void Environment::run()
{
	ros::NodeHandle n;
	ros::NodeHandle pnh("~");
	loadNodeParameters(pnh);

	// Publishers
	ros::Publisher gas_source_pub = n.advertise<visualization_msgs::MarkerArray>("source_visualization", 10);
	ros::Publisher environmnet_pub = n.advertise<visualization_msgs::MarkerArray>("environment_visualization", 100);
	ros::Publisher environmnet_cad_pub = n.advertise<visualization_msgs::MarkerArray>("environment_cad_visualization", 100);

	ros::ServiceServer occupancyMapService = n.advertiseService("gaden_environment/occupancyMap3D", &Environment::occupancyMapServiceCB, this);
	// Subscribers
	preprocessing_done = false;
	ros::Subscriber sub = n.subscribe("preprocessing_done", 1, &Environment::PreprocessingCB, this);


	// 1. ENVIRONMNET AS CAD MODELS
	//-------------------------------
	visualization_msgs::MarkerArray CAD_model_markers;

	for (int i = 0;i < number_of_CAD;i++)
	{
		// CAD model in Collada (.dae) format
		visualization_msgs::Marker cad;
		cad.header.frame_id = fixed_frame;
		cad.header.stamp = ros::Time::now();
		cad.ns = "part_" + std::to_string(i);
		cad.id = i;
		cad.type = visualization_msgs::Marker::MESH_RESOURCE;
		cad.action = visualization_msgs::Marker::ADD;
		cad.mesh_resource = CAD_models[i];
		cad.scale.x = 1.0;
		cad.scale.y = 1.0;
		cad.scale.z = 1.0;
		cad.pose.position.x = 0.0;      //CAD models have the object pose within the file!
		cad.pose.position.y = 0.0;
		cad.pose.position.z = 0.0;
		cad.pose.orientation.x = 0.0;
		cad.pose.orientation.y = 0.0;
		cad.pose.orientation.z = 0.0;
		cad.pose.orientation.w = 1.0;

		//Color (Collada has no color)
		cad.color.r = CAD_color[i][0];
		cad.color.g = CAD_color[i][1];
		cad.color.b = CAD_color[i][2];
		cad.color.a = 1.0;

		//Add Marker to array
		CAD_model_markers.markers.push_back(cad);
	}



	// 2. ENVIRONMNET AS Occupancy3D file
	//------------------------------------
	//Display Environment as an array of Cube markers (Rviz)
	visualization_msgs::MarkerArray environment;
	if (occupancy3D_data != "")
		loadEnvironment(environment);


	// 3. GAS SOURCES
	//----------------
	//Generate Gas Source Markers
	//The shape are cylinders from the floor to the given z_size.
	visualization_msgs::MarkerArray gas_source_markers;

	for (int i = 0;i < number_of_sources;i++)
	{
		visualization_msgs::Marker source;
		source.header.frame_id = fixed_frame;
		source.header.stamp = ros::Time::now();
		source.id = i;
		source.ns = "gas_source_visualization";
		source.action = visualization_msgs::Marker::ADD;
		//source.type = visualization_msgs::Marker::CYLINDER;
		source.type = visualization_msgs::Marker::CUBE;

		source.scale.x = gas_source_scale[i];
		source.scale.y = gas_source_scale[i];
		source.scale.z = gas_source_pos_z[i];
		source.color.r = gas_source_color[i][0];
		source.color.g = gas_source_color[i][1];
		source.color.b = gas_source_color[i][2];
		source.color.a = 1.0;
		source.pose.position.x = gas_source_pos_x[i];
		source.pose.position.y = gas_source_pos_y[i];
		source.pose.position.z = gas_source_pos_z[i] / 2;
		source.pose.orientation.x = 0.0;
		source.pose.orientation.y = 0.0;
		source.pose.orientation.z = 1.0;
		source.pose.orientation.w = 1.0;

		//Add Marker to array
		gas_source_markers.markers.push_back(source);
	}

	// Small sleep to allow RVIZ to startup
	ros::Duration(1.0).sleep();

	//---------------
	//      LOOP
	//---------------
	ros::Rate r(0.3);     //Just to refresh from time to time
	while (ros::ok())
	{
		//Publish CAD Markers
		environmnet_cad_pub.publish(CAD_model_markers);

		// Publish 3D Occupancy
		if (occupancy3D_data != "")
			environmnet_pub.publish(environment);

		//Publish Gas Sources
		gas_source_pub.publish(gas_source_markers);

		ros::spinOnce();
		r.sleep();
	}
}

// ===============================//
//      Load Node parameters      //
// ===============================//
void Environment::loadNodeParameters(ros::NodeHandle private_nh)
{
	private_nh.param<bool>("verbose", verbose, false);
	if (verbose) ROS_INFO("[env] The data provided in the roslaunch file is:");

	private_nh.param<bool>("wait_preprocessing", wait_preprocessing, false);
	if (verbose) ROS_INFO("[env] wait_preprocessing: %u", wait_preprocessing);

	private_nh.param<std::string>("fixed_frame", fixed_frame, "map");
	if (verbose) ROS_INFO("[env] Fixed Frame: %s", fixed_frame.c_str());

	private_nh.param<int>("number_of_sources", number_of_sources, 0);
	if (verbose) ROS_INFO("[env] number_of_sources: %i", number_of_sources);
	gas_source_pos_x.resize(number_of_sources);
	gas_source_pos_y.resize(number_of_sources);
	gas_source_pos_z.resize(number_of_sources);
	gas_source_scale.resize(number_of_sources);
	gas_source_color.resize(number_of_sources);
	for (int i = 0;i < number_of_sources;i++)
	{
		//Get location of soruce for instance (i)
		std::string paramNameX = boost::str(boost::format("source_%i_position_x") % i);
		std::string paramNameY = boost::str(boost::format("source_%i_position_y") % i);
		std::string paramNameZ = boost::str(boost::format("source_%i_position_z") % i);
		std::string scale = boost::str(boost::format("source_%i_scale") % i);
		std::string color = boost::str(boost::format("source_%i_color") % i);

		private_nh.param<double>(paramNameX.c_str(), gas_source_pos_x[i], 0.0);
		private_nh.param<double>(paramNameY.c_str(), gas_source_pos_y[i], 0.0);
		private_nh.param<double>(paramNameZ.c_str(), gas_source_pos_z[i], 0.0);
		private_nh.param<double>(scale.c_str(), gas_source_scale[i], 0.1);
		gas_source_color[i].resize(3);
		private_nh.getParam(color.c_str(), gas_source_color[i]);
		if (verbose) ROS_INFO("[env] Gas_source(%i): pos=[%0.2f %0.2f %0.2f] scale=%.2f color=[%0.2f %0.2f %0.2f]",
			i, gas_source_pos_x[i], gas_source_pos_y[i], gas_source_pos_z[i],
			gas_source_scale[i],
			gas_source_color[i][0], gas_source_color[i][1], gas_source_color[i][2]);
	}

	// CAD MODELS
	//-------------
	//CAD model files
	private_nh.param<int>("number_of_CAD", number_of_CAD, 0);
	if (verbose) ROS_INFO("[env] number_of_CAD: %i", number_of_CAD);

	CAD_models.resize(number_of_CAD);
	CAD_color.resize(number_of_CAD);
	for (int i = 0;i < number_of_CAD;i++)
	{
		//Get location of CAD file for instance (i)
		std::string paramName = boost::str(boost::format("CAD_%i") % i);
		std::string paramColor = boost::str(boost::format("CAD_%i_color") % i);

		private_nh.param<std::string>(paramName.c_str(), CAD_models[i], "");
		CAD_color[i].resize(3);
		private_nh.getParam(paramColor.c_str(), CAD_color[i]);
		if (verbose) ROS_INFO("[env] CAD_models(%i): %s", i, CAD_models[i].c_str());
	}



	//Occupancy 3D gridmap
	//---------------------
	private_nh.param<std::string>("occupancy3D_data", occupancy3D_data, "");
	if (verbose) ROS_INFO("[env] Occupancy3D file location: %s", occupancy3D_data.c_str());
}


//=========================//
// PreProcessing CallBack  //
//=========================//
void Environment::PreprocessingCB(const std_msgs::Bool& b)
{
	preprocessing_done = true;
}




/* Load environment from 3DOccupancy.csv GridMap
 * Loads the environment file containing a description of the simulated environment in the CFD (for the estimation of the wind flows), and displays it.
 * As a general rule, environment files set a value of "0" for a free cell, "1" for a ocuppiedd cell and "2" for outlet.
 * This function creates a cube marker for every occupied cell, with the corresponding dimensions
*/
void Environment::loadEnvironment(visualization_msgs::MarkerArray& env_marker)
{
	// Wait for the GADEN_preprocessin node to finish?
	if (wait_preprocessing)
	{
		while (ros::ok() && !preprocessing_done)
		{
			ros::Duration(0.5).sleep();
			ros::spinOnce();
			if (verbose) ROS_INFO("[environment] Waiting for node GADEN_preprocessing to end.");
		}
	}

	Gaden::ReadResult result = Gaden::readEnvFile(occupancy3D_data, environment);
	if (result == Gaden::ReadResult::NO_FILE)
	{
		ROS_ERROR("No occupancy file provided to environment node!");
		return;
	}
	else if (result == Gaden::ReadResult::READING_FAILED)
	{
		ROS_ERROR("Something went wrong while parsing the file!");
	}

	for (int i = 0; i < environment.description.num_cells.x; i++)
	{
		for (int j = 0; j < environment.description.num_cells.y; j++)
		{
			for (int k = 0; k < environment.description.num_cells.z; k++)
			{
				//Color
				if (!environment.Env[indexFrom3D(i, j, k)])
				{
					//Add a new cube marker for this occupied cell
					visualization_msgs::Marker new_marker;
					new_marker.header.frame_id = fixed_frame;
					new_marker.header.stamp = ros::Time::now();
					new_marker.ns = "environment_visualization";
					new_marker.id = indexFrom3D(i, j, k);                          //unique identifier
					new_marker.type = visualization_msgs::Marker::CUBE;
					new_marker.action = visualization_msgs::Marker::ADD;

					//Center of the cell
					new_marker.pose.position.x = environment.description.min_coord.x + ((i + 0.5) * environment.description.cell_size);
					new_marker.pose.position.y = environment.description.min_coord.y + ((j + 0.5) * environment.description.cell_size);
					new_marker.pose.position.z = environment.description.min_coord.z + ((k + 0.5) * environment.description.cell_size);
					new_marker.pose.orientation.x = 0.0;
					new_marker.pose.orientation.y = 0.0;
					new_marker.pose.orientation.z = 0.0;
					new_marker.pose.orientation.w = 1.0;

					//Size of the cell
					new_marker.scale.x = environment.description.cell_size;
					new_marker.scale.y = environment.description.cell_size;
					new_marker.scale.z = environment.description.cell_size;


					new_marker.color.r = 0.9f;
					new_marker.color.g = 0.1f;
					new_marker.color.b = 0.1f;
					new_marker.color.a = 1.0;
					env_marker.markers.push_back(new_marker);
				}
			}
		}
	}
}

bool Environment::occupancyMapServiceCB(gaden_environment::OccupancyRequest& request, gaden_environment::OccupancyResponse& response)
{
	response.origin.x = environment.description.min_coord.x;
	response.origin.y = environment.description.min_coord.y;
	response.origin.z = environment.description.min_coord.z;

	response.numCellsX = environment.description.num_cells.x;
	response.numCellsY = environment.description.num_cells.y;
	response.numCellsZ = environment.description.num_cells.z;

	response.resolution = environment.description.cell_size;
	response.occupancy = environment.Env;

	return true;
}


