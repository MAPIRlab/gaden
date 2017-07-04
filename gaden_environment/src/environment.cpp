/*
 * The only goal of this Node is to display the simulation environment and gas source location in RVIZ.
 *
 * 1. Loads the simulation environment (usually from CFD in the file format .env), and displays it as RVIZ markers.
 * 2. Displays the Gas-Source Location as two cylinders.
 */

#include "environment/environment.h"



//Load Node parameters
void loadNodeParameters(ros::NodeHandle private_nh)
{

    ROS_INFO("[env] The data provided in the roslaunch file is:");

    //fixed frame
    private_nh.param<std::string>("fixed_frame", fixed_frame, DEFAULT_FIXED_FRAME);
    ROS_INFO("[env] Fixed Frame: %s",fixed_frame.c_str());

    //Gas Sources & their postions (x,y,z)
    private_nh.param<int>("number_of_sources", number_of_sources, 0);
    ROS_INFO("[env] number_of_sources: %i",number_of_sources);

    gas_source_pos_x.resize(number_of_sources);
    gas_source_pos_y.resize(number_of_sources);
    gas_source_pos_z.resize(number_of_sources);
    for(int i=0;i<number_of_sources;i++)
    {
        //Get location of soruce for instance (i)
        std::string paramNameX = boost::str( boost::format("source_%i_position_x") % i);
        std::string paramNameY = boost::str( boost::format("source_%i_position_y") % i);
        std::string paramNameZ = boost::str( boost::format("source_%i_position_z") % i);

        private_nh.param<double>(paramNameX.c_str(), gas_source_pos_x[i], DEFAULT_SOURCE_POS_X);
        private_nh.param<double>(paramNameY.c_str(), gas_source_pos_y[i], DEFAULT_SOURCE_POS_Y);
        private_nh.param<double>(paramNameZ.c_str(), gas_source_pos_z[i], DEFAULT_SOURCE_POS_Z);
        ROS_INFO("[env] Gas_source(%i): [%0.2f %0.2f %0.2f]", i, gas_source_pos_x[i], gas_source_pos_y[i], gas_source_pos_z[i]);
    }

    // CAD MODELS
    //-------------
    //CAD model files
    private_nh.param<int>("number_of_CAD", number_of_CAD, 0);
    ROS_INFO("[env] number_of_CAD: %i",number_of_CAD);

    CAD_models.resize(number_of_CAD);
    for(int i=0;i<number_of_CAD;i++)
    {
        //Get location of CAD file for instance (i)
        std::string paramName = boost::str( boost::format("CAD_%i") % i);

        private_nh.param<std::string>(paramName.c_str(), CAD_models[i], "");
        ROS_INFO("[env] CAD_models(%i): %s",i, CAD_models[i].c_str());
    }



    //Occupancy 3D gridmap
    //---------------------
    private_nh.param<std::string>("occupancy3D_data", occupancy3D_data, "");
    ROS_INFO("[env] Occupancy3D file location: %s",occupancy3D_data.c_str());
}



/* Load environment from 3DOccupancy.csv GridMap
 * Loads the environment file containing a description of the simulated environment in the CFD (for the estimation of the wind flows), and displays it.
 * As a general rule, environment files set a "1" for a ocuppiedd cell, and "0" for a free cell
 * This function creates a cube marker for every occupied cell, with the corresponding dimensions
*/
void loadEnvironment(visualization_msgs::MarkerArray &env_marker)
{
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

        ROS_INFO("[env]Env dimensions (%.2f,%.2f,%.2f)-(%.2f,%.2f,%.2f)",env_min_x, env_min_y, env_min_z, env_max_x, env_max_y, env_max_z );
        ROS_INFO("[env]Env size in cells     (%d,%d,%d) - with cell size %f [m]",env_cells_x,env_cells_y,env_cells_z, cell_size);
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
                    if (f == 1.0)
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

                        //Color (just one color for all occupied cells -_-)
                        new_marker.color.r = 0.8f;
                        new_marker.color.g = 0.8f;
                        new_marker.color.b = 0.8f;
                        new_marker.color.a = 1.0;

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



// MAIN
int main( int argc, char** argv )
{
    //Init
    ros::init(argc, argv, "environment");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");
    ros::Publisher gas_source_pub = n.advertise<visualization_msgs::MarkerArray>("source_visualization", 10);
    ros::Publisher environmnet_pub = n.advertise<visualization_msgs::MarkerArray>("environment_visualization", 100);
    ros::Publisher environmnet_cad_pub = n.advertise<visualization_msgs::MarkerArray>("environment_cad_visualization", 100);


    //Load Parameters
    loadNodeParameters(pnh);

    // 1. ENVIRONMNET AS CAD MODELS
    //-------------------------------
    visualization_msgs::MarkerArray CAD_model_markers;

    //Set pallete of colors
    static const float arr_r[] = {0.92, 0.38, 0.96, 0.64, 0.71};
    static const float arr_g[] = {0.96, 0.92, 0.17, 0.53, 0.71};
    static const float arr_b[] = {0.96, 0.96, 0.3, 0.3, 0.78};
    std::vector<float> color_r (arr_r, arr_r + sizeof(arr_r) / sizeof(arr_r[0]) );
    std::vector<float> color_g (arr_g, arr_g + sizeof(arr_g) / sizeof(arr_g[0]) );
    std::vector<float> color_b (arr_b, arr_b + sizeof(arr_b) / sizeof(arr_b[0]) );

    for (int i=0;i<number_of_CAD;i++)
    {
        //ROS_INFO("Loading CADmodel: %s", CAD_models[i].c_str());

        // CAD model in Collada (.dae) format
        visualization_msgs::Marker cad;
        cad.header.frame_id = fixed_frame;
        cad.header.stamp = ros::Time::now();
        cad.ns = "environment_cad_visualization";
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

        //Color
        cad.color.r = color_r[i];
        cad.color.g = color_g[i];
        cad.color.b = color_b[i];
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
        source.type = visualization_msgs::Marker::CYLINDER;

        source.scale.x = 0.1;
        source.scale.y = 0.1;
        source.scale.z = gas_source_pos_z[i];
        source.color.g = 0.6f;
        source.color.b = 0.4f;
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


    //---------------
    //      LOOP
    //---------------
    ros::Rate r(1);     //1Hz is more than enough
    while (ros::ok())
    {
        //Publish Environment Markers
        environmnet_cad_pub.publish(CAD_model_markers);

        if (occupancy3D_data != "")
            environmnet_pub.publish(environment);

        //Publish Gas Sources
        gas_source_pub.publish(gas_source_markers);

        ros::spinOnce();
        r.sleep();
    }
}
