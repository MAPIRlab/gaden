#ifndef CFilamentSimulator_H
#define CFilamentSimulator_H

#include <omp.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "filament_simulator/filament.h"
#include <stdlib.h>     /* srand, rand */
#include <iostream>
#include <fstream>
#include <random>
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/interprocess/streams/bufferstream.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
class CFilamentSimulator
{
public:
    CFilamentSimulator();
    ~CFilamentSimulator();
    void add_new_filaments(double radius_arround_source);
    void read_wind_snapshot(int idx);
    void update_gas_concentration_from_filaments();
    void update_gas_concentration_from_filament(int fil_i);
    void update_filaments_location();
    void update_filament_location(int i);
    void publish_markers();
    void save_state_to_file();


    //Variables
    int         current_wind_snapshot;
    int         current_simulation_step;
    double      sim_time;
    int         last_saved_step;
    
    //Parameters
    bool        verbose;
    bool        wait_preprocessing;
    bool        preprocessing_done;
    double      max_sim_time;           //(sec) Time tu run this simulation
    int			numSteps;               //Number of gas iterations to simulate
    double		time_step;              //(sec) Time increment between gas snapshots --> Simul_time = snapshots*time_step
    int			numFilaments_sec;       //Num of filaments released per second
    bool        variable_rate;          //If true the number of released filaments would be random(0,numFilaments_sec)
    
    int         filament_stop_steps;    //Number of steps to wait between the release of filaments (to force a patchy plume)
    int         filament_stop_counter;
    
    double		numFilaments_step;      //Num of filaments released per time_step
    double      numFilament_aux;

    int         current_number_filaments;
    int         total_number_filaments; //total number of filaments to use along the simulation (for efficiency -> avoids push_back)
    double      filament_ppm_center;    //[ppm] Gas concentration at the center of the 3D gaussian (filament)
    double      filament_initial_std;   //[cm] Sigma of the filament at t=0-> 3DGaussian shape
    double      filament_growth_gamma;  //[cm²/s] Growth ratio of the filament_std
    double      filament_noise_std;     //STD to add some "variablity" to the filament location
    int			gasType;                //Gas type to simulate
    double      envTemperature;         //Temp in Kelvins
    double      envPressure;            //Pressure in Atm
    int         gasConc_unit;           //Get gas concentration in [molecules/cm3] or [ppm]

    //Wind
    std::string	wind_files_location;    //Location of the wind information
    double      windTime_step;          //(sec) Time increment between wind snapshots
    double      sim_time_last_wind;     //(sec) Simulation Time of the last updated of wind data
    bool        allow_looping;
    int         loop_from_step;
    int         loop_to_step;

    //Enviroment
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

    //Gas Source Location (for releasing the filaments)
    double		gas_source_pos_x;     //[m]
    double		gas_source_pos_y;     //[m]
    double		gas_source_pos_z;     //[m]


    //Results
    int         save_results;           //True or false
    std::string results_location;       //Location for results logfiles
    double      results_time_step;      //(sec) Time increment between saving results
    double      results_min_time;       //(sec) time after which start saving results
    bool wind_finished;
    boost::mutex mtx;


protected:
    void loadNodeParameters();
    void initSimulator();
    void configure3DMatrix(std::vector< double > &A);
    void read_3D_file(std::string filename, std::vector< double > &A, bool hasHeader, bool binary);
    int check_pose_with_environment(double pose_x, double pose_y, double pose_z);
    bool check_environment_for_obstacle(double start_x, double start_y, double start_z, double end_x, double end_y, double end_z);
    double random_number(double min_val, double max_val);
    void preprocessingCB(const std_msgs::Bool& b);

    //Subscriptions & Publishers
    ros::Publisher marker_pub;          //For visualization of the filaments!
    ros::Subscriber prepro_sub;         // In case we require the preprocessing node to finish.

    //Vars
    ros::NodeHandle n;
    std::vector< double > U, V, W, C, Env;
    std::vector<CFilament> filaments;
    visualization_msgs::Marker filament_marker;
    bool wind_notified;    
    int last_wind_idx=-1;
    // SpecificGravity [dimensionless] with respect AIR
    double SpecificGravity[14] = {

        // Molecular gas mass [g/mol]
        // SpecificGravity(Air) = 1 (as reference)
        // Specific gravity is the ratio of the density of a substance to the density of a reference substance; equivalently,
        // it is the ratio of the mass of a substance to the mass of a reference substance for the same given volume.
        1.0378,	  //ethanol   (heavier than air)
        0.5537,	  //methane   (lighter than air)
        0.0696,	  //hydrogen  (lighter than air)
        1.4529,	  //acetone   (heavier than air)

        //To be updated
        1.23,	 //propanol   //gases heavier then air
        2.48,	 //chlorine
        1.31,	 //fluorine
        0.7,	 //neon	   //gases lighter than air
        0.138,   //helium
        0.8, //sewage, biogas
        2.0061, //butane
        0.967, //carbon monoxide
        1.52, //carbon dioxide
        0.89 //smoke
    };

    //Fluid Dynamics
    double filament_initial_vol;
    double env_cell_vol;
    double filament_numMoles;        //Number of moles in a filament (of any gas or air)
    double filament_numMoles_of_gas; //Number of moles of target gas in a filament
    double env_cell_numMoles;        //Number of moles in a cell (3D volume)

    int indexFrom3D(int x, int y, int z);
};

#endif
