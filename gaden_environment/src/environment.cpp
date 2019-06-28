/*
 * The only goal of this Node is to display the simulation environment and gas source location in RVIZ.
 *
 * 1. Loads the simulation environment (usually from CFD in the file format .env), and displays it as RVIZ markers.
 * 2. Displays the Gas-Source Location as two cylinders.
 */

#include "environment/environment.h"


// ===============================//
//      Load Node parameters      //
// ===============================//
void loadNodeParameters(ros::NodeHandle private_nh)
{
    private_nh.param<bool>("verbose", verbose, false);
    if (verbose) ROS_INFO("[env] The data provided in the roslaunch file is:");

    private_nh.param<bool>("wait_preprocessing", wait_preprocessing, false);
    if (verbose) ROS_INFO("[env] wait_preprocessing: %u",wait_preprocessing);

    private_nh.param<std::string>("fixed_frame", fixed_frame, "map");
    if (verbose) ROS_INFO("[env] Fixed Frame: %s",fixed_frame.c_str());

    private_nh.param<int>("number_of_sources", number_of_sources, 0);
    if (verbose) ROS_INFO("[env] number_of_sources: %i",number_of_sources);
    gas_source_pos_x.resize(number_of_sources);
    gas_source_pos_y.resize(number_of_sources);
    gas_source_pos_z.resize(number_of_sources);
    gas_source_scale.resize(number_of_sources);
    gas_source_color.resize(number_of_sources);
    for(int i=0;i<number_of_sources;i++)
    {
        //Get location of soruce for instance (i)
        std::string paramNameX = boost::str( boost::format("source_%i_position_x") % i);
        std::string paramNameY = boost::str( boost::format("source_%i_position_y") % i);
        std::string paramNameZ = boost::str( boost::format("source_%i_position_z") % i);
        std::string scale = boost::str( boost::format("source_%i_scale") % i);
        std::string color = boost::str( boost::format("source_%i_color") % i);

        private_nh.param<double>(paramNameX.c_str(), gas_source_pos_x[i], 0.0);
        private_nh.param<double>(paramNameY.c_str(), gas_source_pos_y[i], 0.0);
        private_nh.param<double>(paramNameZ.c_str(), gas_source_pos_z[i], 0.0);
        private_nh.param<double>(scale.c_str(), gas_source_scale[i], 0.1);
        gas_source_color[i].resize(3);
        private_nh.getParam(color.c_str(),gas_source_color[i]);
        if (verbose) ROS_INFO("[env] Gas_source(%i): pos=[%0.2f %0.2f %0.2f] scale=%.2f color=[%0.2f %0.2f %0.2f]",
                 i, gas_source_pos_x[i], gas_source_pos_y[i], gas_source_pos_z[i],
                 gas_source_scale[i],
                 gas_source_color[i][0],gas_source_color[i][1],gas_source_color[i][2]);
    }

    // CAD MODELS
    //-------------
    //CAD model files
    private_nh.param<int>("number_of_CAD", number_of_CAD, 0);
    if (verbose) ROS_INFO("[env] number_of_CAD: %i",number_of_CAD);

    CAD_models.resize(number_of_CAD);
    CAD_color.resize(number_of_CAD);
    for(int i=0;i<number_of_CAD;i++)
    {
        //Get location of CAD file for instance (i)
        std::string paramName = boost::str( boost::format("CAD_%i") % i);
        std::string paramColor = boost::str( boost::format("CAD_%i_color") % i);

        private_nh.param<std::string>(paramName.c_str(), CAD_models[i], "");
        CAD_color[i].resize(3);
        private_nh.getParam(paramColor.c_str(),CAD_color[i]);
        if (verbose) ROS_INFO("[env] CAD_models(%i): %s",i, CAD_models[i].c_str());
    }



    //Occupancy 3D gridmap
    //---------------------
    private_nh.param<std::string>("occupancy3D_data", occupancy3D_data, "");
    if (verbose) ROS_INFO("[env] Occupancy3D file location: %s",occupancy3D_data.c_str());
}


//=========================//
// PreProcessing CallBack  //
//=========================//
void PreprocessingCB(const std_msgs::Bool& b)
{
    preprocessing_done = true;
}



/* Load environment from 3DOccupancy.csv GridMap
 * Loads the environment file containing a description of the simulated environment in the CFD (for the estimation of the wind flows), and displays it.
 * As a general rule, environment files set a value of "0" for a free cell, "1" for a ocuppiedd cell and "2" for outlet.
 * This function creates a cube marker for every occupied cell, with the corresponding dimensions
*/
void loadEnvironment(visualization_msgs::MarkerArray &env_marker)
{
    // Wait for the GADEN_preprocessin node to finish?
    if( wait_preprocessing )
    {
        while(ros::ok() && !preprocessing_done)
        {
            ros::Duration(0.5).sleep();
            ros::spinOnce();
            if (verbose) ROS_INFO("[environment] Waiting for node GADEN_preprocessing to end.");
        }
	}


    //open file
    std::ifstream infile(occupancy3D_data.c_str());
    std::string line;

    //Read the 4 Header lines
    {
        //Line 1 (min values of environment)
        std::getline(infile, line);
        size_t pos = line.find(" ");
        line.erase(0, pos+1);
        pos = line.find(" ");
        env_min_x = atof(line.substr(0, pos).c_str());
        line.erase(0, pos+1);
        pos = line.find(" ");
        env_min_y = atof(line.substr(0, pos).c_str());
        env_min_z = atof(line.substr(pos+1).c_str());

        //Line 2 (max values of environment)
        std::getline(infile, line);
        pos = line.find(" ");
        line.erase(0, pos+1);
        pos = line.find(" ");
        env_max_x = atof(line.substr(0, pos).c_str());
        line.erase(0, pos+1);
        pos = line.find(" ");
        env_max_y = atof(line.substr(0, pos).c_str());
        env_max_z = atof(line.substr(pos+1).c_str());

        //Line 3 (Num cells on eahc dimension)
        std::getline(infile, line);
        pos = line.find(" ");
        line.erase(0, pos+1);
        pos = line.find(" ");
        env_cells_x = atoi(line.substr(0, pos).c_str());
        line.erase(0, pos+1);
        pos = line.find(" ");
        env_cells_y = atof(line.substr(0, pos).c_str());
        env_cells_z = atof(line.substr(pos+1).c_str());

        //Line 4 cell_size (m)
        std::getline(infile, line);
        pos = line.find(" ");
        cell_size = atof(line.substr(pos+1).c_str());

        if (verbose) ROS_INFO("[env]Env dimensions (%.2f,%.2f,%.2f)-(%.2f,%.2f,%.2f)",env_min_x, env_min_y, env_min_z, env_max_x, env_max_y, env_max_z );
        if (verbose) ROS_INFO("[env]Env size in cells     (%d,%d,%d) - with cell size %f [m]",env_cells_x,env_cells_y,env_cells_z, cell_size);
    }


    //Read file line by line
    int x_idx = 0;
    int y_idx = 0;
    int z_idx = 0;
    int cell_idx = 0;

    while (std::getline(infile, line))
    {
        std::stringstream ss(line);

        if (line == ";")
        {
            //New Z-layer
            z_idx++;
            x_idx = 0;
            y_idx = 0;
        }
        else
        {   //New line with constant x_idx and all the y_idx values
            while (!ss.fail())
            {
                double f;
                ss >> f;        //get one double value
                if (!ss.fail())
                {
                    // Occupie cell or Outlet
                    if (f == 1.0 || f == 2.0)
                    {
                        //Add a new cube marker for this occupied cell
                        visualization_msgs::Marker new_marker;
                        new_marker.header.frame_id = fixed_frame;
                        new_marker.header.stamp = ros::Time::now();
                        new_marker.ns = "environment_visualization";
                        new_marker.id = cell_idx;                          //unique identifier
                        new_marker.type = visualization_msgs::Marker::CUBE;
                        new_marker.action = visualization_msgs::Marker::ADD;

                        //Center of the cell
                        new_marker.pose.position.x = env_min_x + ( (x_idx + 0.5) * cell_size);
                        new_marker.pose.position.y = env_min_y + ( (y_idx + 0.5) * cell_size);
                        new_marker.pose.position.z = env_min_z + ( (z_idx + 0.5) * cell_size);
                        new_marker.pose.orientation.x = 0.0;
                        new_marker.pose.orientation.y = 0.0;
                        new_marker.pose.orientation.z = 0.0;
                        new_marker.pose.orientation.w = 1.0;

                        //Size of the cell
                        new_marker.scale.x = cell_size;
                        new_marker.scale.y = cell_size;
                        new_marker.scale.z = cell_size;

                        //Color
                        if (f == 1.0)
                        {
                            new_marker.color.r = 0.8f;
                            new_marker.color.g = 0.8f;
                            new_marker.color.b = 0.8f;
                            new_marker.color.a = 1.0;
                        }
                        else
                        {
                            new_marker.color.r = 0.9f;
                            new_marker.color.g = 0.1f;
                            new_marker.color.b = 0.1f;
                            new_marker.color.a = 1.0;
                        }

                        env_marker.markers.push_back(new_marker);
                    }
                    y_idx++;
                    cell_idx++;
                }
            }

            //Line has ended
            x_idx++;
            y_idx = 0;
        }
    }

    //End of file.
}




// ===============================//
//              MAIN              //
// ===============================//
int main( int argc, char** argv )
{
    //Init
    ros::init(argc, argv, "environment");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");

    //Load Parameters
    loadNodeParameters(pnh);

    // Publishers
    ros::Publisher gas_source_pub = n.advertise<visualization_msgs::MarkerArray>("source_visualization", 10);
    ros::Publisher environmnet_pub = n.advertise<visualization_msgs::MarkerArray>("environment_visualization", 100);
    ros::Publisher environmnet_cad_pub = n.advertise<visualization_msgs::MarkerArray>("environment_cad_visualization", 100);

    // Subscribers
    preprocessing_done  =false;
    ros::Subscriber sub = n.subscribe("preprocessing_done", 1, PreprocessingCB);


    // 1. ENVIRONMNET AS CAD MODELS
    //-------------------------------
    visualization_msgs::MarkerArray CAD_model_markers;

    for (int i=0;i<number_of_CAD;i++)
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

    for (int i=0;i<number_of_sources;i++)
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
        source.pose.position.z = gas_source_pos_z[i]/2;
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
