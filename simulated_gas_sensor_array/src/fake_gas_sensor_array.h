#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <olfaction_msgs/gas_sensor_array.h>
#include <gaden_player/GasPosition.h>
#include <boost/format.hpp>

#include <cstdlib>
#include <math.h>
#include <vector>
#include <fstream>
#include <iostream>


//Gas Types
#define	ETHANOL_ID		0
#define	METHANE_ID		1
#define	HYDROGEN_ID		2
#define	PROPANOL_ID		3
#define	CHLORIDE_ID		4
#define	FLURORINE_ID	5
#define	ACETONE_ID		6
#define	NEON_ID			7
#define	HELIUM_ID		8
#define	HOTAIR_ID		9

//Sensor Types
#define	TGS2620_ID		0
#define	TGS2600_ID		1
#define	TGS2611_ID		2
#define TGS2610_ID      3
#define TGS2612_ID      4
#define PID_ID          30

	
// Parameters
int                 num_sensors;                    //number of sensors in the array
std::vector<int>	sensor_models;
double              pub_rate;
std::string         topic_id;
std::string         frame_id;
std::string         fixed_frame;
bool                use_PID_correction_factors;


//Parameters to model the MOX response
struct gas_sensor
{
    bool    first_reading;                 //First reading is set to baseline always
    float   RS_R0;                        //Ideal sensor response based on sensitivity
    float   sensor_output;                //MOX model response
    float   previous_sensor_output;       //The response in (t-1)
};
std::vector<gas_sensor>	sensor_array;

// Vars
int     ch_id;                          //Chemical ID
bool    notified;                      //to notifiy about erros just once

//functions:
void  loadNodeParameters(ros::NodeHandle private_nh);
float simulate_mox_as_line_loglog(gaden_player::GasPositionResponse GT_gas_concentrations, int s_idx);
float simulate_pid(gaden_player::GasPositionResponse GT_gas_concentrations);



//------------------------ SENSOR CHARACTERIZATION PARAMS ----------------------------------//

std::string labels[5] = {"TGS2620", "TGS2600", "TGS2611", "TGS2610", "TGS2612"};
float R0[5] = {3000, 50000, 3740, 3740, 4500};      //[Ohms] Reference resistance (see datasheets)

//Time constants (Rise, Decay)
float tau_value[5][7][2] =      //5 sensors, 7 gases , 2 Time Constants
{
    {  //TGS2620
    {2.96, 15.71},  //ethanol
    {2.96, 15.71},  //methane
    {2.96, 15.71},  //hydrogen
    {2.96, 15.71},  //propanol
    {2.96, 15.71},  //chlorine
    {2.96, 15.71},  //fluorine
    {2.96, 15.71}   //Acetone
    },

    {  //TGS2600
    {4.8, 18.75},   //ethanol
    {4.8, 18.75},   //methane
    {4.8, 18.75},   //hydrogen
    {4.8, 18.75},   //propanol
    {4.8, 18.75},   //chlorine
    {4.8, 18.75},   //fluorine
    {4.8, 18.75}    //Acetone
    },

    {  //TGS2611
    {3.44, 6.35},   //ethanol
    {3.44, 6.35},   //methane
    {3.44, 6.35},   //hydrogen
    {3.44, 6.35},   //propanol
    {3.44, 6.35},   //chlorine
    {3.44, 6.35},   //fluorine
    {3.44, 6.35}    //Acetone
    },

    {  //TGS2610
    {3.44, 6.35},   //ethanol
    {3.44, 6.35},   //methane
    {3.44, 6.35},   //hydrogen
    {3.44, 6.35},   //propanol
    {3.44, 6.35},   //chlorine
    {3.44, 6.35},   //fluorine
    {3.44, 6.35}    //Acetone
    },

    {  //TGS2612
    {3.44, 6.35},   //ethanol
    {3.44, 6.35},   //methane
    {3.44, 6.35},   //hydrogen
    {3.44, 6.35},   //propanol
    {3.44, 6.35},   //chlorine
    {3.44, 6.35},   //fluorine
    {3.44, 6.35}    //Acetone
    }
};

// MOX sensitivity. Extracted from datasheets and curve fitting
//--------------------------------------------------------------
float Sensitivity_Air[5] = {21, 1, 8.8, 10.3, 19.5};      //RS/R0 when exposed to clean air (datasheet)

// RS/R0 = A*conc^B (a line in the loglog scale)
float sensitivity_lineloglog[5][7][2]={   //5 Sensors, 7 Gases, 2 Constants: A, B
    {  //TGS2620    
    {62.32, -0.7155},   //Ethanol
    {120.6, -0.4877},   //Methane
    {24.45, -0.5546},   //Hydrogen
       {120.6, -0.4877},   //propanol (To review)
       {120.6, -0.4877},   //chlorine (To review)
       {120.6, -0.4877},   //fluorine (To review)
       {120.6, -0.4877}    //Acetone (To review)
    },

    {  //TGS2600    
    {0.6796, -0.3196},   //ethanol
    {1.018, -0.07284},   //methane
    {0.6821, -0.3532},    //hydrogen
       {1.018, -0.07284},   //propanol (To review)
       {1.018, -0.07284},   //chlorine (To review)
       {1.018, -0.07284},   //fluorine (To review)
       {1.018, -0.07284}    //Acetone (To review)
    },

    {  //TGS2611    
    {51.11, -0.3658},    //ethanol
    {38.46, -0.4289},    //methane
    {41.3, -0.3614},     //hydrogen
       {38.46, -0.4289},   //propanol (To review)
       {38.46, -0.4289},   //chlorine (To review)
       {38.46, -0.4289},   //fluorine (To review)
       {38.46, -0.4289}    //Acetone (To review)
    },

    {  //TGS2610
    {106.1, -0.5008},     //ethanol
    {63.91, -0.5372},     //methane
    {66.78, -0.4888},     //hydrogen
       {63.91, -0.5372},   //propanol (To review)
       {63.91, -0.5372},   //chlorine (To review)
       {63.91, -0.5372},   //fluorine (To review)
       {63.91, -0.5372}    //Acetone (To review)
    },

    {  //TGS2612
    {31.35, -0.09115},   //ethanol
    {146.2, -0.5916},    //methane
    {19.5, 0.0},         //hydrogen
       {146.2, -0.5916},   //propanol (To review)
       {146.2, -0.5916},   //chlorine (To review)
       {146.2, -0.5916},   //fluorine (To review)
       {146.2, -0.5916}    //Acetone (To review)
    }
};


//PID correction factors for gas concentration
//--------------------------------------------
//Ethanol, Methane, Hydrogen, Propanol, Chlorine, Fluorine, Acetone
// http://www.intlsensor.com/pdf/pidcorrectionfactors.pdf
// Here we simulate a lamp of 11.7eV to increase the range of detectable gases
// A 0.0 means the PID is not responsive to that gas
float PID_correction_factors[7] = {10.47, 0.0, 0.0, 2.7, 1.0, 0.0, 1.4};
