#ifndef Preprocess_H
#define Preprocess_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>
#include <fstream>
#include <stdlib.h> 
#include <vector>
#include <sstream>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <boost/format.hpp>
#include <queue>
#include <stack>

//dimensions of the enviroment [m]
double env_min_x; 
double env_min_y; 
double env_min_z;
double env_max_x; 
double env_max_y; 
double env_max_z;
double roundFactor;
//length of the sides of the cell [m]
double cell_size;

//some auxiliary stuff
double min_val(double x, double y, double z);
double max_val(double x, double y, double z);

//check if the plane goes through the cell
bool planeIntersects(Eigen::Vector3f n, float d, Eigen::Vector3f query_point); 

//proyect the center of the cell onto the plane of the triangle
bool pointInTriangle(const Eigen::Vector3f& query_point,
                     const Eigen::Vector3f& triangle_vertex_0,
                     const Eigen::Vector3f& triangle_vertex_1,
                     const Eigen::Vector3f& triangle_vertex_2);

//check if the triangle is on a plane that's parallel to yz or xz
bool parallel (std::vector<double> &vec);

//these are for printing the results on files
void printEnv(std::string filename, std::vector<std::vector<std::vector<int> > > env, int scale);
void printWind(std::vector<std::vector<std::vector<double> > > U,
                std::vector<std::vector<std::vector<double> > > V,
                std::vector<std::vector<std::vector<double> > > W, std::string filename);

//these are for parsing the stl files and generating a 3D occupancy map
void occupy(std::vector<std::vector<std::vector<int> > >& env,
            std::vector<std::vector<std::vector<double> > > &points, 
            std::vector<std::vector<double> > &normals, int val);
void parse(std::string filename, std::vector<std::vector<std::vector<int> > >& env, int val);
void findDimensions(std::string filename);

//translates the point cloud to the gaden wind format
void openFoam_to_gaden(std::string filename, std::vector<std::vector<std::vector<int> > >& env);

#endif
