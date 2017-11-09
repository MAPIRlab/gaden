#ifndef CFilamentSimulator_H
#define CFilamentSimulator_H


#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

#include "filament_simulator/filament.h"
#include <stdlib.h>     /* srand, rand */
#include <fstream>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#include <gaden_msgs/SimulationIterationRequest.h>
#include <ros/callback_queue.h>
#include <boost/thread.hpp>

//
// Type definitions for a easier gaussian random number generation
//
typedef boost::normal_distribution<double> NormalDistribution;
typedef boost::mt19937 RandomGenerator;
typedef boost::variate_generator<RandomGenerator&, \
                       NormalDistribution> GaussianGenerator;

class CFilamentSimulator
{
public:
    CFilamentSimulator();
    ~CFilamentSimulator();
    void add_new_filaments(double radius_arround_source);
    void read_wind_snapshot(int idx);
    void update_gas_concentration_from_filaments();
    void update_filaments_location();
    void publish_markers();
    void save_state_to_file();


    //Variables
    int         current_wind_snapshot;
    int         current_simulation_step;
    double      sim_time;
    int         last_saved_step;

    //Parameters
    double      max_sim_time;           //(sec) Time tu run this simulation
    int			numSteps;               //Number of gas iterations to simulate
    double		time_step;              //(sec) Time increment between gas snapshots --> Simul_time = snapshots*time_step
    int			numFilaments_sec;       //Num of filaments released per second
    int			numFilaments_step;      //Num of filaments released per time_step
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
    double      restuls_time_step;      //(sec) Time increment between saving results


    void initAsyncSpinner();
    void shutdownAsyncSpinner();

protected:
    void loadNodeParameters();
    void initSimulator();
    void configure3DMatrix(std::vector< std::vector< std::vector<double> > > &A);
    void read_3D_file(std::string filename, std::vector< std::vector< std::vector<double> > > &A, bool hasHeader);
    int check_pose_with_environment(double pose_x, double pose_y, double pose_z);
    bool check_environment_for_obstacle(double start_x, double start_y, double start_z, double end_x, double end_y, double end_z);
    double random_number(double min_val, double max_val);
    
    bool requestSimulationStep(gaden_msgs::SimulationIterationRequest::Request&, gaden_msgs::SimulationIterationRequest::Response&);

    void rosLoop();

    //Subscriptions & Publishers
    ros::Publisher marker_pub;          //For visualization of the filaments!

    //Vars
    ros::NodeHandle n;
    ros::ServiceServer simulationIterationService;

    boost::shared_ptr<ros::AsyncSpinner> asyncSpinner;
    boost::shared_ptr<boost::thread> workerThread;
    
    std::vector< std::vector< std::vector<double> > > U, V, W, C, Env;
    std::vector<CFilament> filaments;
    visualization_msgs::Marker filament_marker;
    bool wind_notified;    

    // SpecificGravity [dimensionless] with respect AIR
    double SpecificGravity[10];

    //Fluid Dynamics
    double filament_initial_vol;
    double env_cell_vol;
    double filament_numMoles;        //Number of moles in a filament (of any gas or air)
    double filament_numMoles_of_gas; //Number of moles of target gas in a filament
    double env_cell_numMoles;        //Number of moles in a cell (3D volume)


};

#endif
