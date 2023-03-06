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

#include "fake_gas_sensor_array.h"

int main( int argc, char** argv )
{
    ros::init(argc, argv, "fake_gas_sensor_aray");
	ros::NodeHandle n;
    ros::NodeHandle pn("~");

    //Read parameters
    loadNodeParameters(pn);
	
    //Publishers
    ros::Publisher enose_pub = n.advertise<olfaction_msgs::gas_sensor_array>(topic_id, 500);
	
    //Service to request gas concentration
    ros::ServiceClient client = n.serviceClient<gaden_player::GasPosition>("/odor_value");


    //Configure sensor_array
    sensor_array.resize(num_sensors);
    for (int i=0;i<num_sensors; i++)
    {
        sensor_array[i].first_reading = true;
    }

    // Loop
    tf::TransformListener listener;
    ros::Rate r(pub_rate);
    notified = false;
    while (ros::ok())
    {
        //Vars
        tf::StampedTransform transform;
        bool know_sensor_pose = true;

        //Get pose of the sensor_aray in the /map reference
        try
        {
          listener.lookupTransform(fixed_frame.c_str(), frame_id.c_str(),
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

            // Get Gas concentration at current position (for each gas present)
            // Service request to the simulator
            gaden_player::GasPosition srv;
            srv.request.x.push_back(x_pos);
            srv.request.y.push_back(y_pos);
            srv.request.z.push_back(z_pos);            
            if (client.call(srv))
            {
                /*
                for (int i=0; i<srv.response.gas_type.size(); i++)
                {
                    ROS_INFO("[FakeMOX] %s:%.4f at (%.2f,%.2f,%.2f)",srv.response.gas_type[i].c_str(), srv.response.gas_conc[i],srv.request.x, srv.request.y, srv.request.z );
                }
                */

                olfaction_msgs::gas_sensor_array enose_msg;
                enose_msg.header.frame_id = frame_id;
                enose_msg.header.stamp = ros::Time::now();
                //For each sensor in the array, simulate its response
                for (int s=0; s<num_sensors; s++)
                {
                    //Simulate Gas_Sensor response given this GT values of the concentration!
                    olfaction_msgs::gas_sensor sensor_msg;
                    sensor_msg.header.frame_id = frame_id;
                    sensor_msg.header.stamp = ros::Time::now();
                    switch (sensor_models[s])
                    {
                    case 0: //MOX TGS2620
                        sensor_msg.technology = sensor_msg.TECH_MOX;
                        sensor_msg.manufacturer = sensor_msg.MANU_FIGARO;
                        sensor_msg.mpn = sensor_msg.MPN_TGS2620;
                        sensor_msg.raw_units = sensor_msg.UNITS_OHM;
                        sensor_msg.raw = simulate_mox_as_line_loglog(srv.response, s);
                        sensor_msg.raw_air = Sensitivity_Air[sensor_models[s]]*R0[sensor_models[s]];
                        sensor_msg.calib_A = sensitivity_lineloglog[sensor_models[s]][0][0];  //Calib for Ethanol
                        sensor_msg.calib_B = sensitivity_lineloglog[sensor_models[s]][0][1];  //Calib for Ethanol
                        break;
                    case 1:  //MOX TGS2600
                        sensor_msg.technology = sensor_msg.TECH_MOX;
                        sensor_msg.manufacturer = sensor_msg.MANU_FIGARO;
                        sensor_msg.mpn = sensor_msg.MPN_TGS2600;
                        sensor_msg.raw_units = sensor_msg.UNITS_OHM;
                        sensor_msg.raw = simulate_mox_as_line_loglog(srv.response, s);
                        sensor_msg.raw_air = Sensitivity_Air[sensor_models[s]]*R0[sensor_models[s]];
                        sensor_msg.calib_A = sensitivity_lineloglog[sensor_models[s]][0][0];  //Calib for Ethanol
                        sensor_msg.calib_B = sensitivity_lineloglog[sensor_models[s]][0][1];  //Calib for Ethanol
                        break;
                    case 2:  //MOX TGS2611
                        sensor_msg.technology = sensor_msg.TECH_MOX;
                        sensor_msg.manufacturer = sensor_msg.MANU_FIGARO;
                        sensor_msg.mpn = sensor_msg.MPN_TGS2611;
                        sensor_msg.raw_units = sensor_msg.UNITS_OHM;
                        sensor_msg.raw = simulate_mox_as_line_loglog(srv.response, s);
                        sensor_msg.raw_air = Sensitivity_Air[sensor_models[s]]*R0[sensor_models[s]];
                        sensor_msg.calib_A = sensitivity_lineloglog[sensor_models[s]][0][0];  //Calib for Ethanol
                        sensor_msg.calib_B = sensitivity_lineloglog[sensor_models[s]][0][1];  //Calib for Ethanol
                        break;
                    case 3:  //MOX TGS2610
                        sensor_msg.technology = sensor_msg.TECH_MOX;
                        sensor_msg.manufacturer = sensor_msg.MANU_FIGARO;
                        sensor_msg.mpn = sensor_msg.MPN_TGS2610;
                        sensor_msg.raw_units = sensor_msg.UNITS_OHM;
                        sensor_msg.raw = simulate_mox_as_line_loglog(srv.response, s);
                        sensor_msg.raw_air = Sensitivity_Air[sensor_models[s]]*R0[sensor_models[s]];
                        sensor_msg.calib_A = sensitivity_lineloglog[sensor_models[s]][0][0];  //Calib for Ethanol
                        sensor_msg.calib_B = sensitivity_lineloglog[sensor_models[s]][0][1];  //Calib for Ethanol
                        break;
                    case 4:  //MOX TGS2612
                        sensor_msg.technology = sensor_msg.TECH_MOX;
                        sensor_msg.manufacturer = sensor_msg.MANU_FIGARO;
                        sensor_msg.mpn = sensor_msg.MPN_TGS2612;
                        sensor_msg.raw_units = sensor_msg.UNITS_OHM;
                        sensor_msg.raw = simulate_mox_as_line_loglog(srv.response, s);
                        sensor_msg.raw_air = Sensitivity_Air[sensor_models[s]]*R0[sensor_models[s]];
                        sensor_msg.calib_A = sensitivity_lineloglog[sensor_models[s]][0][0];  //Calib for Ethanol
                        sensor_msg.calib_B = sensitivity_lineloglog[sensor_models[s]][0][1];  //Calib for Ethanol
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

                    //append sensor observation to the array
                    enose_msg.sensors.push_back(sensor_msg);
                }//end for each sensor in the array


                //Publish simulated enose (Array of sensors)
                enose_pub.publish(enose_msg);
                notified = false;
            }
            else
            {
                if (!notified)
                {
                    ROS_WARN("[fake_gas_sensor_array] Cannot read Gas Concentrations from GADEN simulator.");
                    notified = true;
                }
            }
        }

        ros::spinOnce();
        r.sleep();
    }
}




// Simulate MOX response: Sensitivity + Dynamic response
// RS = R0*( A * conc^B )
// This method employes a curve fitting based on a line in the loglog scale to set the sensitivity
float simulate_mox_as_line_loglog(gaden_player::GasPositionResponse GT_gas_concentrations, int s_idx)
{
    if (sensor_array[s_idx].first_reading)
    {
        //Init sensor to its Baseline lvl
        sensor_array[s_idx].sensor_output = Sensitivity_Air[sensor_models[s_idx]];    //RS_R0 value at air
        sensor_array[s_idx].previous_sensor_output = sensor_array[s_idx].sensor_output;
        sensor_array[s_idx].first_reading = false;
    }
    else
    {        
        //1. Set Sensor Output based on gas concentrations (gas type dependent)
        //---------------------------------------------------------------------
        // RS/R0 = A*conc^B (a line in the loglog scale)
        float resistance_variation = 0.0;

        //Handle multiple gases
        for (int i=0; i<GT_gas_concentrations.positions[0].concentration.size(); i++)
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
            sensor_array[s_idx].RS_R0 = sensitivity_lineloglog[sensor_models[s_idx]][gas_id][0] * pow(GT_gas_concentrations.positions[0].concentration[i], sensitivity_lineloglog[sensor_models[s_idx]][gas_id][1]);

            //Ensure we never overpass the baseline level (max allowed)
            if (sensor_array[s_idx].RS_R0 > Sensitivity_Air[sensor_models[s_idx]])
                sensor_array[s_idx].RS_R0= Sensitivity_Air[sensor_models[s_idx]];

            //Increment with respect the Baseline
            resistance_variation += Sensitivity_Air[sensor_models[s_idx]] - sensor_array[s_idx].RS_R0;
        }

        //Calculate final RS_R0 given the final resistance variation
        sensor_array[s_idx].RS_R0 = Sensitivity_Air[sensor_models[s_idx]] - resistance_variation;

        //Ensure a minimum sensor resitance
        if (sensor_array[s_idx].RS_R0 <= 0.0)
            sensor_array[s_idx].RS_R0 = 0.01;



        //2. Simulate transient response (dynamic behaviour, tau_r and tau_d)
        //---------------------------------------------------------------------
        float tau;
        if (sensor_array[s_idx].RS_R0 < sensor_array[s_idx].previous_sensor_output)  //rise
            tau = tau_value[sensor_models[s_idx]][0][0];
        else //decay
            tau = tau_value[sensor_models[s_idx]][0][1];

        // Use a low pass filter
        //alpha value = At/(tau+At)
        float alpha = (1/pub_rate) / (tau+(1/pub_rate));

        //filtered response (uses previous estimation):
        sensor_array[s_idx].sensor_output = (alpha*sensor_array[s_idx].RS_R0) + (1-alpha)*sensor_array[s_idx].previous_sensor_output;

        //Update values
        sensor_array[s_idx].previous_sensor_output = sensor_array[s_idx].sensor_output;
    }

    // Return Sensor response for current time instant as the Sensor Resistance in Ohms
    return (sensor_array[s_idx].sensor_output * R0[sensor_models[s_idx]]);
}



// Simulate PID response : Weighted Sum of all gases
float simulate_pid(gaden_player::GasPositionResponse GT_gas_concentrations)
{
    //Handle multiple gases
    float accumulated_conc = 0.0;
    for (int i=0; i<GT_gas_concentrations.positions[0].concentration.size(); i++)
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
                accumulated_conc += GT_gas_concentrations.positions[0].concentration[i] / PID_correction_factors[gas_id];
        }
        else
            accumulated_conc += GT_gas_concentrations.positions[0].concentration[i];
    }
    return accumulated_conc;
}


//Load Sensor parameters
void loadNodeParameters(ros::NodeHandle private_nh)
{
    //Num sensors in the array
    private_nh.param<int>("num_sensors", num_sensors, 1);

    //Sensor models
    sensor_models.resize(num_sensors);
    for (int i=0;i<num_sensors; i++)
    {
        //Get model of sensor (i)
        std::string paramName = boost::str( boost::format("sensor_model_%i") % i);
        private_nh.param<int>(paramName.c_str(),sensor_models[i], TGS2620_ID);
    }

    //Publication rate (Hz)
    private_nh.param<double>("pub_rate", pub_rate, 5.0);

    //sensor_array_topic_id
    private_nh.param<std::string>("topic_id", topic_id, "/enose");

    //sensor_array_frame_id
    private_nh.param<std::string>("frame_id", frame_id, "enose_frame");

	//fixed frame
    private_nh.param<std::string>("fixed_frame", fixed_frame, "/map");

    //PID_correction_factors
    private_nh.param<bool>("use_PID_correction_factors", use_PID_correction_factors, false);
}









