#ifndef EVNIRONMENT_ENVIRONMENT_H_
#define EVNIRONMENT_ENVIRONMENT_H_
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <resource_retriever/retriever.h>
#include <cmath>
#include <fstream>

#define		NODE_NAME 							"environment"
#define 	DEFAULT_SOURCE_POS_X				0.0
#define 	DEFAULT_SOURCE_POS_Y				0.0
#define 	DEFAULT_SOURCE_POS_Z				0.5
#define		DEFAULT_ENVIRONMENT_DATA			""
#define		DEFAULT_FIXED_FRAME					"/map"
#define	DEFAULT_AREA_X						31
#define	DEFAULT_AREA_Y						16
#define	DEFAULT_AREA_Z						6
#define	DEFAULT_AREA_CELL_SIZE				1

//Gas Sources
int                         number_of_sources;
std::vector<double>         gas_source_pos_x;
std::vector<double>         gas_source_pos_y;
std::vector<double>         gas_source_pos_z;

//CAD models
int                         number_of_CAD;
std::vector<std::string>    CAD_models;

//Environment 3D
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



//Methods
void loadNodeParameters(ros::NodeHandle);
void loadEnvironment(visualization_msgs::MarkerArray &env_marker);
#endif

