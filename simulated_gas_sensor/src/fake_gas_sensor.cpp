/*-------------------------------------------------------------------------------
 * This node simulates the response of a MOX gas sensor given the GT gas concentration
 * of the gases it is exposed to (request to simulation_player or dispersion_simulation)
 * - Gas concentration should be given in [ppm]
 * - The Pkg response can be set to:  Resistance of the sensor (Rs), Resistance-ratio (Rs/R0), or Voltage (0-5V)
 * - Sensitivity to different gases is set based on manufacter datasheet
 * - Time constants for the dynamic response are set based on real experiments
 *
 * - Response to mixture of gases is set based on  datasheet.
 * -----------------------------------------------------------------------------------------------*/

#include "fake_gas_sensor.h"

int main( int argc, char** argv )
{
    ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n;
    ros::NodeHandle pn("~");

    //Read parameters
    loadNodeParameters(pn);
	
    //Publishers
    ros::Publisher sensor_read_pub = n.advertise<olfaction_msgs::gas_sensor>("Sensor_reading", 500);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("Sensor_display", 100);
	
    //Service to request gas concentration
    ros::ServiceClient client = n.serviceClient<gaden_player::GasPosition>("/odor_value");
	

    //Init Visualization data (marker)
    //---------------------------------
    // sensor = sphere
    // conector = stick from the floor to the sensor
	visualization_msgs::Marker sensor,connector;
	sensor.header.frame_id = input_fixed_frame.c_str();
	sensor.ns = "sensor_visualization";	
    sensor.action = visualization_msgs::Marker::ADD;
	sensor.type = visualization_msgs::Marker::SPHERE;
    sensor.id = 0;
    sensor.scale.x = 0.1;
    sensor.scale.y = 0.1;
    sensor.scale.z = 0.1;
    sensor.color.r = 2.0f;
	sensor.color.g = 1.0f;
    sensor.color.a = 1.0;
	
	connector.header.frame_id = input_fixed_frame.c_str();
	connector.ns  = "sensor_visualization";
	connector.action = visualization_msgs::Marker::ADD;
    connector.type = visualization_msgs::Marker::CYLINDER;
    connector.id = 1;
	connector.scale.x = 0.1;
	connector.scale.y = 0.1;
	connector.color.a  = 1.0;
	connector.color.r = 1.0f;
	connector.color.b = 1.0f;
	connector.color.g = 1.0f;


    // Loop
	tf::TransformListener listener;
    node_rate = 5;  //Hz
    ros::Rate r(node_rate);
    first_reading = true;
    notified = false;
    while (ros::ok())
    {
        //Vars
        tf::StampedTransform transform;
        bool know_sensor_pose = true;

        //Get pose of the sensor in the /map reference
        try
        {
          listener.lookupTransform(input_fixed_frame.c_str(), input_sensor_frame.c_str(),
                                   ros::Time(0), transform);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s",ex.what());
            know_sensor_pose = false;
            ros::Duration(1.0).sleep();
        }

        if (know_sensor_pose)
        {
            //Current sensor pose
            float x_pos = transform.getOrigin().x();
            float y_pos = transform.getOrigin().y();
            float z_pos = transform.getOrigin().z();

            // Get Gas concentration at current position (of each gas present)
            // Service request to the simulator
            gaden_player::GasPosition srv;
            srv.request.x = x_pos;
            srv.request.y = y_pos;
            srv.request.z = z_pos;            
            if (client.call(srv))
            {
/*
                for (int i=0; i<srv.response.gas_type.size(); i++)
                {
                    ROS_INFO("[FakeMOX] %s:%.4f at (%.2f,%.2f,%.2f)",srv.response.gas_type[i].c_str(), srv.response.gas_conc[i],srv.request.x, srv.request.y, srv.request.z );
                }

*/

                //Simulate Gas_Sensor response given this GT values of the concentration!
                olfaction_msgs::gas_sensor sensor_msg;
                sensor_msg.header.frame_id = input_sensor_frame;
                sensor_msg.header.stamp = ros::Time::now();
                switch (input_sensor_model)
                {
                case 0: //MOX TGS2620
                    sensor_msg.technology = sensor_msg.TECH_MOX;
                    sensor_msg.manufacturer = sensor_msg.MANU_FIGARO;
                    sensor_msg.mpn = sensor_msg.MPN_TGS2620;
                    sensor_msg.raw_units = sensor_msg.UNITS_OHM;
                    sensor_msg.raw = simulate_mox_as_line_loglog(srv.response);
                    sensor_msg.raw_air = Sensitivity_Air[input_sensor_model]*R0[input_sensor_model];
                    sensor_msg.calib_A = sensitivity_lineloglog[input_sensor_model][0][0];  //Calib for Ethanol
                    sensor_msg.calib_B = sensitivity_lineloglog[input_sensor_model][0][1];  //Calib for Ethanol
                    break;
                case 1:  //MOX TGS2600
                    sensor_msg.technology = sensor_msg.TECH_MOX;
                    sensor_msg.manufacturer = sensor_msg.MANU_FIGARO;
                    sensor_msg.mpn = sensor_msg.MPN_TGS2600;
                    sensor_msg.raw_units = sensor_msg.UNITS_OHM;
                    sensor_msg.raw = simulate_mox_as_line_loglog(srv.response);
                    sensor_msg.raw_air = Sensitivity_Air[input_sensor_model]*R0[input_sensor_model];
                    sensor_msg.calib_A = sensitivity_lineloglog[input_sensor_model][0][0];  //Calib for Ethanol
                    sensor_msg.calib_B = sensitivity_lineloglog[input_sensor_model][0][1];  //Calib for Ethanol
                    break;
                case 2:  //MOX TGS2611
                    sensor_msg.technology = sensor_msg.TECH_MOX;
                    sensor_msg.manufacturer = sensor_msg.MANU_FIGARO;
                    sensor_msg.mpn = sensor_msg.MPN_TGS2611;
                    sensor_msg.raw_units = sensor_msg.UNITS_OHM;
                    sensor_msg.raw = simulate_mox_as_line_loglog(srv.response);
                    sensor_msg.raw_air = Sensitivity_Air[input_sensor_model]*R0[input_sensor_model];
                    sensor_msg.calib_A = sensitivity_lineloglog[input_sensor_model][0][0];  //Calib for Ethanol
                    sensor_msg.calib_B = sensitivity_lineloglog[input_sensor_model][0][1];  //Calib for Ethanol
                    break;
                case 3:  //MOX TGS2610
                    sensor_msg.technology = sensor_msg.TECH_MOX;
                    sensor_msg.manufacturer = sensor_msg.MANU_FIGARO;
                    sensor_msg.mpn = sensor_msg.MPN_TGS2610;
                    sensor_msg.raw_units = sensor_msg.UNITS_OHM;
                    sensor_msg.raw = simulate_mox_as_line_loglog(srv.response);
                    sensor_msg.raw_air = Sensitivity_Air[input_sensor_model]*R0[input_sensor_model];
                    sensor_msg.calib_A = sensitivity_lineloglog[input_sensor_model][0][0];  //Calib for Ethanol
                    sensor_msg.calib_B = sensitivity_lineloglog[input_sensor_model][0][1];  //Calib for Ethanol
                    break;
                case 4:  //MOX TGS2612
                    sensor_msg.technology = sensor_msg.TECH_MOX;
                    sensor_msg.manufacturer = sensor_msg.MANU_FIGARO;
                    sensor_msg.mpn = sensor_msg.MPN_TGS2612;
                    sensor_msg.raw_units = sensor_msg.UNITS_OHM;
                    sensor_msg.raw = simulate_mox_as_line_loglog(srv.response);
                    sensor_msg.raw_air = Sensitivity_Air[input_sensor_model]*R0[input_sensor_model];
                    sensor_msg.calib_A = sensitivity_lineloglog[input_sensor_model][0][0];  //Calib for Ethanol
                    sensor_msg.calib_B = sensitivity_lineloglog[input_sensor_model][0][1];  //Calib for Ethanol
                    break;


                case 30:  //PID miniRaeLite
                    sensor_msg.technology = sensor_msg.TECH_PID;
                    sensor_msg.manufacturer = sensor_msg.MANU_RAE;
                    sensor_msg.mpn = sensor_msg.MPN_MINIRAELITE;
                    sensor_msg.raw_units = sensor_msg.UNITS_PPM;
                    sensor_msg.raw = simulate_pid(srv.response);
                    sensor_msg.raw_air = 0.0;
                    sensor_msg.calib_A = 0.0;
                    sensor_msg.calib_B = 0.0;
                    break;
                default:
                    break;
                }


                //Publish simulated sensor reading
                sensor_read_pub.publish(sensor_msg);
                notified = false;
            }
            else
            {
                if (!notified)
                {
                    ROS_WARN("[fake_gas_sensor] Cannot read Gas Concentrations from simulator.");
                    notified = true;
                }
            }


            //Publish RVIZ sensor pose
            sensor.header.stamp = ros::Time::now();
            sensor.pose.position.x = x_pos;
            sensor.pose.position.y = y_pos;
            sensor.pose.position.z = z_pos;
            marker_pub.publish(sensor);
            connector.header.stamp = ros::Time::now();
            connector.scale.z = z_pos;
            connector.pose.position.x = x_pos;
            connector.pose.position.y = y_pos;
            connector.pose.position.z = float(z_pos)/2;
            marker_pub.publish(connector);
        }

        ros::spinOnce();
        r.sleep();
    }
}


// Simulate MOX response: Sensitivity + Dynamic response
// RS = R0*( A * conc^B )
// This method employes a curve fitting based on a line in the loglog scale to set the sensitivity
float simulate_mox_as_line_loglog(gaden_player::GasPositionResponse GT_gas_concentrations)
{
    if (first_reading)
    {
        //Init sensor to its Baseline lvl
        sensor_output = Sensitivity_Air[input_sensor_model];    //RS_R0 value at air
        previous_sensor_output = sensor_output;
        first_reading = false;
    }
    else
    {        
        //1. Set Sensor Output based on gas concentrations (gas type dependent)
        //---------------------------------------------------------------------
        // RS/R0 = A*conc^B (a line in the loglog scale)
        float resistance_variation = 0.0;

        //Handle multiple gases
        for (int i=0; i<GT_gas_concentrations.gas_conc.size(); i++)
        {
            int gas_id;
            if (!strcmp(GT_gas_concentrations.gas_type[i].c_str(),"ethanol"))
                gas_id = 0;
            else if (!strcmp(GT_gas_concentrations.gas_type[i].c_str(),"methane"))
                gas_id = 1;
            else if (!strcmp(GT_gas_concentrations.gas_type[i].c_str(),"hydrogen"))
                gas_id = 2;
            else if (!strcmp(GT_gas_concentrations.gas_type[i].c_str(),"propanol"))
                gas_id = 3;
            else if (!strcmp(GT_gas_concentrations.gas_type[i].c_str(),"chlorine"))
                gas_id = 4;
            else if (!strcmp(GT_gas_concentrations.gas_type[i].c_str(),"fluorine"))
                gas_id = 5;
            else if (!strcmp(GT_gas_concentrations.gas_type[i].c_str(),"acetone"))
                gas_id = 6;
            else
            {
                ROS_ERROR("[fake_mox] MOX response is not configured for this gas type!");
                return 0.0;
            }

            //JUST FOR VIDEO DEMO
            /*
            if (input_sensor_model == 0)
            {
                GT_gas_concentrations.gas_conc[i] *= 10;
            }
            else if (input_sensor_model ==2)
            {
                GT_gas_concentrations.gas_conc[i] *= 20;
            }
            */

            //Value of RS/R0 for the given gas and concentration
            RS_R0 = sensitivity_lineloglog[input_sensor_model][gas_id][0] * pow(GT_gas_concentrations.gas_conc[i], sensitivity_lineloglog[input_sensor_model][gas_id][1]);

            //Ensure we never overpass the baseline level (max allowed)
            if (RS_R0 > Sensitivity_Air[input_sensor_model])
                RS_R0= Sensitivity_Air[input_sensor_model];

            //Increment with respect the Baseline
            resistance_variation += Sensitivity_Air[input_sensor_model] - RS_R0;
        }

        //Calculate final RS_R0 given the final resistance variation
        RS_R0 = Sensitivity_Air[input_sensor_model] - resistance_variation;

        //Ensure a minimum sensor resitance
        if (RS_R0 <= 0.0)
            RS_R0 = 0.01;



        //2. Simulate transient response (dynamic behaviour, tau_r and tau_d)
        //---------------------------------------------------------------------
        float tau;
        if (RS_R0 < previous_sensor_output)  //rise
            tau = tau_value[input_sensor_model][0][0];
        else //decay
            tau = tau_value[input_sensor_model][0][1];

        // Use a low pass filter
        //alpha value = At/(tau+At)
        float alpha = (1/node_rate) / (tau+(1/node_rate));

        //filtered response (uses previous estimation):
        sensor_output = (alpha*RS_R0) + (1-alpha)*previous_sensor_output;

        //Update values
        previous_sensor_output = sensor_output;
    }

    // Return Sensor response for current time instant as the Sensor Resistance in Ohms
    return (sensor_output * R0[input_sensor_model]);
}



// Simulate PID response : Weighted Sum of all gases
float simulate_pid(gaden_player::GasPositionResponse GT_gas_concentrations)
{
    //Handle multiple gases
    float accumulated_conc = 0.0;
    for (int i=0; i<GT_gas_concentrations.gas_conc.size(); i++)
    {
        if (use_PID_correction_factors)
        {
            int gas_id;
            if (!strcmp(GT_gas_concentrations.gas_type[i].c_str(),"ethanol"))
                gas_id = 0;
            else if (!strcmp(GT_gas_concentrations.gas_type[i].c_str(),"methane"))
                gas_id = 1;
            else if (!strcmp(GT_gas_concentrations.gas_type[i].c_str(),"hydrogen"))
                gas_id = 2;
            else
            {
                ROS_ERROR("[fake_PID] PID response is not configured for this gas type!");
                return 0.0;
            }
            if (PID_correction_factors[gas_id] != 0)
                accumulated_conc += GT_gas_concentrations.gas_conc[i] / PID_correction_factors[gas_id];
        }
        else
            accumulated_conc += GT_gas_concentrations.gas_conc[i];
    }
    return accumulated_conc;
}

// ===============================//
//      Load Node Parameters      //
// ===============================//
void loadNodeParameters(ros::NodeHandle private_nh)
{	
    //fixed frame
    private_nh.param<std::string>("fixed_frame", input_fixed_frame, "map");

    //Sensor Model
    private_nh.param<int>("sensor_model", input_sensor_model, DEFAULT_SENSOR_MODEL);

    //sensor_frame
    private_nh.param<std::string>("sensor_frame", input_sensor_frame, DEFAULT_SENSOR_FRAME);


    //PID_correction_factors
    private_nh.param<bool>("use_PID_correction_factors", use_PID_correction_factors, false);




    ROS_INFO("The data provided in the roslaunch file is:");
	ROS_INFO("Sensor model: %d",input_sensor_model);
	ROS_INFO("Fixed frame: %s",input_fixed_frame.c_str());
    ROS_INFO("Sensor frame: %s",input_sensor_frame.c_str());    
}









