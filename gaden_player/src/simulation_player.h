#include "gaden_common/Vector3.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <gaden_msgs/srv/gas_position.hpp>
#include <gaden_msgs/srv/wind_position.hpp>
#include <gaden_msgs/msg/gas_in_cell.hpp>

#include <cstdlib>
#include <math.h>
#include <vector>
#include <string>

#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>

#include <gaden_common/ReadEnvironment.h>

struct Filament
{
public:
    float x, y, z, sigma;
    Filament(float a, float b, float c, float d)
    {
        x = a;
        y = b;
        z = c;
        sigma = d;
    }
};

class Simulation;

class Player : public rclcpp::Node
{
public:
    Player();
    void run();

private:
    // ----------------------  MAIN--------------------//

    // Parameters
    float player_freq;
    int num_simulators;
    bool verbose;
    std::vector<std::string> simulation_data;
    std::vector<std_msgs::msg::ColorRGBA> gas_display_colors;
    std::vector<Simulation> player_instances; // To handle N simulations at a time.

    int initial_iteration, loop_from_iteration, loop_to_iteration;
    bool allow_looping;
    std::string occupancyFile;

    // Visualization
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
    visualization_msgs::msg::Marker mkr_gas_points; // We will create an array of particles according to cell concentration
    // functions:
    void loadNodeParameters();
    void init_all_simulation_instances();
    void load_all_data_from_logfiles(int sim_iteration);
    void display_current_gas_distribution();

    gaden_msgs::msg::GasInCell get_all_gases_single_cell(float x, float y, float z, const std::vector<std::string>& gas_types);
    bool get_gas_value_srv(gaden_msgs::srv::GasPosition::Request::SharedPtr req, gaden_msgs::srv::GasPosition::Response::SharedPtr res);
    bool get_wind_value_srv(gaden_msgs::srv::WindPosition::Request::SharedPtr req, gaden_msgs::srv::WindPosition::Response::SharedPtr res);
};

// CLASS for every simulation to run. If two gas sources are needed, just create 2 instances!
class Simulation
{
public:
    Simulation(std::string filepath, bool load_wind_info, std::string occupancy_filePath, std_msgs::msg::ColorRGBA display_color);
    ~Simulation();

    std::string gas_type;
    std::string simulation_filename;
    std::string occupancyFile;
    gaden::Environment environment;
    float source_pos_x, source_pos_y, source_pos_z;
    std_msgs::msg::ColorRGBA gas_display_color;

    bool load_wind_data;
    std::vector<gaden::Vector3> wind;
    bool first_reading;
    int last_wind_idx = -1;

    float total_moles_in_filament;
    float num_moles_all_gases_in_cm3;
    std::vector<Filament> activeFilaments;

    // methods
    void configure_environment();
    
    void load_data_from_logfile(int sim_iteration);
    void load_logfile_version_1(std::stringstream& decompressed);
    void load_logfile_version_pre_2_6(std::stringstream& decompressed);
    void load_logfile_current(std::stringstream& decompressed);
    void load_wind_file_pre_2_6(int wind_index);
    void load_wind_file_current(int wind_index);

    float get_gas_concentration(float x, float y, float z);
    float concentration_from_filament(float x, float y, float z, Filament fil);
    bool check_environment_for_obstacle(float start_x, float start_y, float start_z, float end_x, float end_y, float end_z);
    int check_pose_with_environment(float pose_x, float pose_y, float pose_z);

    gaden::Vector3 get_wind_value(const gaden::Vector3& location);
    void get_concentration_as_markers(visualization_msgs::msg::Marker& mkr_points);


    int indexFrom3D(int x, int y, int z);

    std::string gasTypesByCode[14] = {"ethanol", "methane", "hydrogen", "propanol", "chlorine",       "flurorine",       "acetone",
                                      "neon",    "helium",  "biogas",   "butane",   "carbon dioxide", "carbon monoxide", "smoke"};
};