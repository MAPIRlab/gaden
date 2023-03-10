#pragma once
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <resource_retriever/retriever.h>
#include <cmath>
#include <vector>
#include <fstream>
#include <boost/format.hpp>
#include <gaden_environment/Occupancy.h>

class Environment
{
    public:
        void run();
    
    private:

    //Gas Sources
    int                                 number_of_sources;
    std::vector<double>                 gas_source_pos_x;
    std::vector<double>                 gas_source_pos_y;
    std::vector<double>                 gas_source_pos_z;
    std::vector<double>                 gas_source_scale;
    std::vector< std::vector<double> >  gas_source_color;

    //CAD models
    int                                 number_of_CAD;
    std::vector<std::string>            CAD_models;
    std::vector< std::vector<double> >  CAD_color;

    //Environment 3D
    std::vector<uint8_t> Env;
    std::string occupancy3D_data;       //Location of the 3D Occupancy GridMap of the environment
    std::string	fixed_frame;            //Frame where to publish the markers
    int			env_cells_x;            //cells
    int 		env_cells_y;            //cells
    int 		env_cells_z;            //cells
    double      env_min_x;              //[m]
    double      env_max_x;              //[m]
    double      env_min_y;              //[m]
    double      env_max_y;              //[m]
    double      env_min_z;              //[m]
    double      env_max_z;              //[m]
    double		cell_size;              //[m]

    bool        verbose;
    bool        wait_preprocessing;
    bool        preprocessing_done;

    //Methods
    void loadNodeParameters(ros::NodeHandle);
    void loadEnvironment(visualization_msgs::MarkerArray &env_marker);
    void readEnvFile();

    bool occupancyMapServiceCB(gaden_environment::OccupancyRequest& request, gaden_environment::OccupancyResponse& response);
    int indexFrom3D(int x, int y, int z);
    void PreprocessingCB(const std_msgs::Bool& b);

};

