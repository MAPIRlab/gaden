#ifndef CFilamentSimulator_H
#define CFilamentSimulator_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include "filament_simulator/filament.h"
#include "gaden_common/Vector3.h"

#include <omp.h>
#include <stdlib.h> /* srand, rand */
#include <boost/format.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/interprocess/streams/bufferstream.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>
#include <gaden_common/ReadEnvironment.h>

class CFilamentSimulator : public rclcpp::Node
{
public:
    CFilamentSimulator();
    ~CFilamentSimulator();
    void add_new_filaments(float radius_arround_source);
    void read_wind_snapshot(int idx);
    void update_filaments_location();
    void update_filament_location(int i);
    void publish_markers();
    void save_state_to_file();

    // Variables
    int current_wind_snapshot;
    int current_simulation_step;
    float sim_time;
    int last_saved_step;
    float last_saved_timestamp;

    // Parameters
    bool verbose;
    bool wait_preprocessing;
    bool preprocessing_done;
    float max_sim_time;  //(sec) Time tu run this simulation
    int numSteps;         // Number of gas iterations to simulate
    float time_step;     //(sec) Time increment between gas snapshots --> Simul_time = snapshots*time_step
    int numFilaments_sec; // Num of filaments released per second
    bool variable_rate;   // If true the number of released filaments would be random(0,numFilaments_sec)

    int filament_stop_steps; // Number of steps to wait between the release of filaments (to force a patchy plume)
    int filament_stop_counter;

    float numFilaments_step; // Num of filaments released per time_step
    float numFilament_aux;

    int current_number_filaments;
    int total_number_filaments;   // total number of filaments to use along the simulation (for efficiency -> avoids push_back)
    float filament_ppm_center;   //[ppm] Gas concentration at the center of the 3D gaussian (filament)
    float filament_initial_std;  //[cm] Sigma of the filament at t=0-> 3DGaussian shape
    float filament_growth_gamma; //[cm²/s] Growth ratio of the filament_std
    float filament_noise_std;    // STD to add some "variablity" to the filament location
    int gasType;                  // Gas type to simulate
    float envTemperature;        // Temp in Kelvins
    float envPressure;           // Pressure in Atm
    int gasConc_unit;             // Get gas concentration in [molecules/cm3] or [ppm]

    // Wind
    std::string wind_files_location; // Location of the wind information
    float windTime_step;            //(sec) Time increment between wind snapshots
    float sim_time_last_wind;       //(sec) Simulation Time of the last updated of wind data
    bool allow_looping;
    int loop_from_step;
    int loop_to_step;

    // Enviroment
    std::string occupancy3D_filepath; // Location of the 3D Occupancy GridMap of the environment
    std::string fixed_frame;      // Frame where to publish the markers
    gaden::Environment environment;

    // Gas Source Location (for releasing the filaments)
    gaden::Vector3 gas_source_pos; //[m]

    // Results
    int save_results;             // True or false
    std::string results_location; // Location for results logfiles
    float results_time_step;     //(sec) Time increment between saving results
    float results_min_time;      //(sec) time after which start saving results
    bool wind_finished;
    boost::mutex mtx;

private:
    void loadNodeParameters();
    void initSimulator();

    bool parseWindFile(int idx);
    bool parseOldWindFiles(int idx);
    gaden::CellState check_pose_with_environment(float pose_x, float pose_y, float pose_z);
    gaden::CellState moveFilament(CFilament& filament, float end_x, float end_y, float end_z);
    float random_number(float min_val, float max_val);
    void preprocessingCB(const std_msgs::msg::Bool::SharedPtr b);

    // Subscriptions & Publishers
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub; // For visualization of the filaments!
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr prepro_sub;          // In case we require the preprocessing node to finish.

    // Vars
    std::vector<gaden::Vector3> wind;
    std::vector<CFilament> filaments;
    visualization_msgs::msg::Marker filament_marker;
    bool wind_notified;
    int last_wind_idx = -1;
    // SpecificGravity [dimensionless] with respect AIR
    float SpecificGravity[14] = {

        // Molecular gas mass [g/mol]
        // SpecificGravity(Air) = 1 (as reference)
        // Specific gravity is the ratio of the density of a substance to the density of a reference substance; equivalently,
        // it is the ratio of the mass of a substance to the mass of a reference substance for the same given volume.
        1.0378, // ethanol   (heavier than air)
        0.5537, // methane   (lighter than air)
        0.0696, // hydrogen  (lighter than air)
        1.4529, // acetone   (heavier than air)

        // To be updated
        1.23,   // propanol   //gases heavier then air
        2.48,   // chlorine
        1.31,   // fluorine
        0.7,    // neon	   //gases lighter than air
        0.138,  // helium
        0.8,    // sewage, biogas
        2.0061, // butane
        0.967,  // carbon monoxide
        1.52,   // carbon dioxide
        0.89    // smoke
    };

    // Fluid Dynamics
    float filament_initial_vol;
    float env_cell_vol;
    float filament_numMoles;        // Number of moles in a filament (of any gas or air)
    float filament_numMoles_of_gas; // Number of moles of target gas in a filament
    float env_cell_numMoles;        // Number of moles in a cell (3D volume)

    size_t indexFrom3D(int x, int y, int z);
};

#endif
