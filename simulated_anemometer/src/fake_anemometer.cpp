
#include "fake_anemometer.h"

#include <stdlib.h>     /* srand, rand */
#include <time.h>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>


typedef boost::normal_distribution<double> NormalDistribution;
typedef boost::mt19937 RandomGenerator;
typedef boost::variate_generator<RandomGenerator&, \
                       NormalDistribution> GaussianGenerator;
                       
                       

int main( int argc, char** argv )
{
	ros::init(argc, argv, "simulated_anemometer");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	srand (time(NULL));


	//Read parameters
	loadNodeParameters(pn);

	
	//Publishers
	//ros::Publisher sensor_read_pub = n.advertise<std_msgs::Float32MultiArray>("WindSensor_reading", 500);
	ros::Publisher sensor_read_pub = n.advertise<olfaction_msgs::anemometer>("WindSensor_reading", 500);
	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("WindSensor_display", 100);

	
	//Service to request wind values to simulator
    ros::ServiceClient client = n.serviceClient<gaden_player::WindPosition>("/wind_value");
	tf::TransformListener tf_;


	// Init Visualization data (marker)
	//----------------------------------------------------------------
	// sensor = sphere
	// conector = stick from the floor to the sensor
	visualization_msgs::Marker sensor;
	visualization_msgs::Marker connector;
	visualization_msgs::Marker connector_inv;
	visualization_msgs::Marker wind_point;
	visualization_msgs::Marker wind_point_inv;
	
	sensor.header.frame_id = input_fixed_frame.c_str();
	sensor.ns = "sensor_visualization";	
	sensor.action = visualization_msgs::Marker::ADD;
	sensor.type = visualization_msgs::Marker::SPHERE;
	sensor.id = 0;
	sensor.scale.x = 0.1;
	sensor.scale.y = 0.1;
	sensor.scale.z = 0.1;
	sensor.color.r = 0.0f;
	sensor.color.g = 0.0f;
	sensor.color.b = 1.0f;
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

	// Init Marker: arrow to display the wind direction measured.
	wind_point.header.frame_id = input_sensor_frame.c_str();
	wind_point.action = visualization_msgs::Marker::ADD;
	wind_point.ns = "measured_wind";
	wind_point.type = visualization_msgs::Marker::ARROW;

	// Init Marker: arrow to display the inverted wind direction measured.	
	wind_point_inv.header.frame_id = input_sensor_frame.c_str();
	wind_point_inv.action = visualization_msgs::Marker::ADD;
	wind_point_inv.ns = "measured_wind_inverted";
	wind_point_inv.type = visualization_msgs::Marker::ARROW;


	// LOOP
	//----------------------------------------------------------------
	tf::TransformListener listener;
    ros::Rate r(2);
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

			// Get Wind vectors (u,v,w) at current position
			// Service request to the simulator
            gaden_player::WindPosition srv;
			srv.request.x = x_pos;
			srv.request.y = y_pos;
			srv.request.z = z_pos;
			float u,v,w;
			olfaction_msgs::anemometer anemo_msg;
			if (client.call(srv))
			{
                double wind_speed;
                double wind_direction;

				//GT Wind vector Value (u,v,w)[m/s]
                //From OpenFoam this is the DownWind direction in the map
				u = (float)srv.response.u;
				v = (float)srv.response.v;
				w = (float)srv.response.w;
                wind_speed = sqrt(pow(u,2)+pow(v,2));

                float downWind_direction_map;
                if (u !=0 || v!=0)
                    downWind_direction_map = atan2(v,u);
                else
                    downWind_direction_map = 0.0;


                if (!use_map_ref_system)
                {
                    // (IMPORTANT) Follow standards on wind measurement (real anemometers):
                    //return the upwind direction in the anemometer reference system
                    //range [-pi,pi]
                    //positive to the right, negative to the left (opposed to ROS poses :s)

                    float upWind_direction_map = angles::normalize_angle(downWind_direction_map + 3.14159);

                    //Transform from map ref_system to the anemometer ref_system using TF
                    geometry_msgs::PoseStamped anemometer_upWind_pose, map_upWind_pose;
                    try
                    {
                        map_upWind_pose.header.frame_id = input_fixed_frame.c_str();
                        map_upWind_pose.pose.position.x = 0.0;
                        map_upWind_pose.pose.position.y = 0.0;
                        map_upWind_pose.pose.position.z = 0.0;
                        map_upWind_pose.pose.orientation = tf::createQuaternionMsgFromYaw(upWind_direction_map);

                        tf_.transformPose(input_sensor_frame.c_str(), map_upWind_pose, anemometer_upWind_pose);
                    }
                    catch(tf::TransformException &ex)
                    {
                        ROS_ERROR("FakeAnemometer - %s - Error: %s", __FUNCTION__, ex.what());
                    }

                    double upwind_direction_anemo = tf::getYaw(anemometer_upWind_pose.pose.orientation);
                    wind_direction = upwind_direction_anemo;
                }
                else
                {
                    // for simulations
                    wind_direction = downWind_direction_map;
                }


                // Adding Noise
                static RandomGenerator rng(static_cast<unsigned> (time(0)));
                NormalDistribution gaussian_dist(0.0,noise_std);
                GaussianGenerator generator(rng, gaussian_dist);
                wind_direction = wind_direction + generator();
                wind_speed = wind_speed + generator();


                //Publish 2D Anemometer readings
                //------------------------------
				anemo_msg.header.stamp = ros::Time::now();
                if (use_map_ref_system)
                    anemo_msg.header.frame_id = input_fixed_frame.c_str();
                else
                    anemo_msg.header.frame_id = input_sensor_frame.c_str();
				anemo_msg.sensor_label = "Fake_Anemo";
                anemo_msg.wind_direction = wind_direction;  //rad
                anemo_msg.wind_speed = wind_speed;	 //m/s
				//Publish fake_anemometer reading (m/s)
				sensor_read_pub.publish(anemo_msg);



                //Add wind marker ARROW for Rviz (2D) --> Upwind
                /*
				wind_point.header.stamp = ros::Time::now();
				wind_point.points.clear();
				wind_point.id = 1;  //unique identifier for each arrow
				wind_point.pose.position.x = 0.0;
				wind_point.pose.position.y = 0.0;
				wind_point.pose.position.z = 0.0;
				wind_point.pose.orientation = tf::createQuaternionMsgFromYaw(wind_direction_with_noise);
				wind_point.scale.x = 2*sqrt(pow(u,2)+pow(v,2));	  //arrow lenght
				wind_point.scale.y = 0.1;	  //arrow width
				wind_point.scale.z = 0.1;	  //arrow height
				wind_point.color.r = 0.0;
				wind_point.color.g = 1.0;
				wind_point.color.b = 0.0;
				wind_point.color.a = 1.0;
				marker_pub.publish(wind_point);
                */
				
                //Add inverted wind marker --> DownWind
				wind_point_inv.header.stamp = ros::Time::now();
				wind_point_inv.points.clear();
				wind_point_inv.id = 1;  //unique identifier for each arrow
				wind_point_inv.pose.position.x = 0.0;
				wind_point_inv.pose.position.y = 0.0;
				wind_point_inv.pose.position.z = 0.0;
                wind_point_inv.pose.orientation = tf::createQuaternionMsgFromYaw(wind_direction+3.1416);
				wind_point_inv.scale.x = 2*sqrt(pow(u,2)+pow(v,2));	  //arrow lenght
				wind_point_inv.scale.y = 0.1;	  //arrow width
				wind_point_inv.scale.z = 0.1;	  //arrow height
				wind_point_inv.color.r = 0.0;
				wind_point_inv.color.g = 1.0;
				wind_point_inv.color.b = 0.0;
				wind_point_inv.color.a = 1.0;
				marker_pub.publish(wind_point_inv);

                notified = false;
			}
			else
			{
                if (!notified)
                {
                    ROS_WARN("[fake_anemometer] Cannot read Wind Vector from simulated data.");
                    notified = true;
                }
			}


            //Publish RVIZ sensor pose (a sphere)
            /*
			sensor.header.stamp = ros::Time::now();
			sensor.pose.position.x = x_pos;
			sensor.pose.position.y = y_pos;
			sensor.pose.position.z = z_pos;
			marker_pub.publish(sensor);
            */

            // PUBLISH ANEMOMETER Stick
            /*
			connector.header.stamp = ros::Time::now();
			connector.scale.z = z_pos;
			connector.pose.position.x = x_pos;
			connector.pose.position.y = y_pos;
			connector.pose.position.z = float(z_pos)/2;
            marker_pub.publish(connector);
            */
		}

		ros::spinOnce();
		r.sleep();
	}
}


//Load Sensor parameters
void loadNodeParameters(ros::NodeHandle private_nh)
{
	//sensor_frame
	private_nh.param<std::string>("sensor_frame", input_sensor_frame, "/anemometer_link");

	//fixed frame
	private_nh.param<std::string>("fixed_frame", input_fixed_frame, "/map");
	
	//Noise
	private_nh.param<double>("noise_std", noise_std, 0.1);

    //What ref system to use for publishing measurements
    private_nh.param<bool>("use_map_ref_system", use_map_ref_system, false);
	
	ROS_INFO("[fake anemometer]: wind noise: %f", noise_std);
	
}









