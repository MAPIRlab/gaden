#include <ros/ros.h>
//#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
//#include <std_msgs/Float32.h>
//#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <olfaction_msgs/anemometer.h>
#include <gaden_player/WindPosition.h>
#include <angles/angles.h>

#include <cstdlib>
#include <math.h>
#include <vector>
#include <fstream>
#include <iostream>



// Sensor Parameters
std::string		input_sensor_frame;
std::string		input_fixed_frame;
double			noise_std;
bool                    use_map_ref_system;


// Vars
bool first_reading = true;
bool notified = false;


//functions:
void  loadNodeParameters(ros::NodeHandle private_nh);
