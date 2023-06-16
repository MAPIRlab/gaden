#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <gaden_player/srv/gas_position.hpp>
#include <gaden_player/srv/wind_position.hpp>
#include <gaden_player/msg/gas_in_cell.hpp>

#include <cstdlib>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <fstream>
#include <string>
#include <time.h>
#include <map>

#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/copy.hpp>

#include <gaden_common/ReadEnvironment.h>

struct Filament{
    public:
        double x, y, z, sigma;
        Filament(double a, double b, double c, double d){
            x=a;
            y=b;
            z=c;
            sigma=d;
        }
};

class sim_obj;

class Player : public rclcpp::Node
{
public:
    Player();
    void run();

private:
    // ----------------------  MAIN--------------------//

    // Parameters
    double                          player_freq;
    int                             num_simulators;
    bool                            verbose;
    std::vector<std::string>        simulation_data;
    std::vector<sim_obj>            player_instances;          //To handle N simulations at a time.

    int                             initial_iteration, loop_from_iteration, loop_to_iteration;
    bool                            allow_looping;
    std::string occupancyFile;

    //Visualization
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr  marker_pub;
    visualization_msgs::msg::Marker      mkr_gas_points;                  //We will create an array of particles according to cell concentration
    //functions:
    void loadNodeParameters();
    void init_all_simulation_instances();
    void load_all_data_from_logfiles(int sim_iteration);
    void display_current_gas_distribution();

    gaden_player::msg::GasInCell get_all_gases_single_cell(float x, float y, float z, const std::vector<std::string>& gas_types);
    bool get_gas_value_srv(gaden_player::srv::GasPosition::Request::SharedPtr req, gaden_player::srv::GasPosition::Response::SharedPtr res);
    bool get_wind_value_srv(gaden_player::srv::WindPosition::Request::SharedPtr req, gaden_player::srv::WindPosition::Response::SharedPtr res);
};


// CLASS for every simulation to run. If two gas sources are needed, just create 2 instances!
class sim_obj
{
public:
    sim_obj(std::string filepath, bool load_wind_info, rclcpp::Logger logger, std::string occupancy_filepath);
    ~sim_obj();

    rclcpp::Logger m_logger;
    std::string     gas_type;
    std::string     simulation_filename;
    std::string     occupancyFile;
    Gaden::EnvironmentDescription envDesc;
    double          source_pos_x, source_pos_y, source_pos_z;
    
    bool            load_wind_data;
    std::vector<double> C;  //3D Gas concentration
    std::vector<double> U;  //3D Wind U
    std::vector<double> V;  //3D Wind V
    std::vector<double> W;  //3D Wind W
    bool            first_reading;

    bool            filament_log;
    double total_moles_in_filament;
    double num_moles_all_gases_in_cm3;
    std::map<int, Filament> activeFilaments;

    //methods
    void configure_environment();
    void load_data_from_logfile(int sim_iteration);
    void load_ascii_file(std::stringstream &decompressed);
    void load_binary_file(std::stringstream &decompressed);
    double get_gas_concentration(float x, float y, float z);
    double concentration_from_filament(float x, float y, float z, Filament fil);
    bool check_environment_for_obstacle(double start_x, double start_y, double start_z,
                                                    double   end_x, double   end_y, double end_z);
    int check_pose_with_environment(double pose_x, double pose_y, double pose_z);

    void get_wind_value(float x, float y, float z, double &u, double &v, double &w);
    void get_concentration_as_markers(visualization_msgs::msg::Marker &mkr_points);
    
    void read_headers(std::stringstream &inbuf, std::string &line);
    void load_wind_file(int wind_index);
    int last_wind_idx=-1;
    void read_concentration_line(std::string line);

    int indexFrom3D(int x, int y, int z);

    std::string gasTypesByCode[14] = {
        "ethanol",
        "methane",
        "hydrogen",
        "propanol",
        "chlorine",
        "flurorine",
        "acetone",
        "neon",
        "helium",
        "biogas",
        "butane",
        "carbon dioxide",
        "carbon monoxide",
        "smoke"
    };
};