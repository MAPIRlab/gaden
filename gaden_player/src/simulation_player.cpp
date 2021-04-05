/*--------------------------------------------------------------------------------
 * Pkg for playing the simulation results of the "filament_simulator" pkg.
 * It allows to run on real time, and provide services to simulated sensors (gas, wind)
 * It supports loading several simulations at a time, which allows multiple gas sources and gas types
 * It also generates a point cloud representing the gas concentration [ppm] on the 3D environment
 --------------------------------------------------------------------------------*/

#include "simulation_player.h"

//--------------- SERVICES CALLBACKS----------------------//
bool get_gas_value_srv(gaden_player::GasPosition::Request  &req, gaden_player::GasPosition::Response &res)
{
    //ROS_INFO("[Player] Request for gas concentration at location [%.2f, %.2f, %.2f]m",req.x, req.y, req.z);

    //Get all gas concentrations and gas types (from all instances)
    for (int i=0;i<num_simulators; i++)
        player_instances[i].get_gas_concentration(req.x, req.y, req.z, srv_response_gas_types[i], srv_response_gas_concs[i]);

    //Return gas concentration for each gas_type.
    //If we have multiple instances with the same gas, then add their concentrations.
    std::map<std::string,double> mymap;
    for (int i=0;i<num_simulators; i++)
    {
        if (mymap.find(srv_response_gas_types[i]) == mymap.end() )
            mymap[srv_response_gas_types[i]] = srv_response_gas_concs[i];
        else
            mymap[srv_response_gas_types[i]] += srv_response_gas_concs[i];
    }

    //Configure Response
    res.gas_conc.clear();
    res.gas_type.clear();
    for (std::map<std::string,double>::iterator it=mymap.begin(); it!=mymap.end(); ++it)
    {
        res.gas_type.push_back(it->first);
        res.gas_conc.push_back(it->second);
    }

    return true;
}


bool get_wind_value_srv(gaden_player::WindPosition::Request  &req, gaden_player::WindPosition::Response &res)
{
    //Since the wind fields are identical among different instances, return just the information from instance[0]
    player_instances[0].get_wind_value(req.x, req.y, req.z, res.u, res.v, res.w);
    return true;
}



//------------------------ MAIN --------------------------//

int main( int argc, char** argv )
{
    ros::init(argc, argv, "simulation_player");
	ros::NodeHandle n;
    ros::NodeHandle pn("~");

    //Read Node Parameters
    loadNodeParameters(pn);	
	
    //Publishers
    marker_pub = n.advertise<visualization_msgs::Marker>("Gas_Distribution", 1);

    //Services offered
    ros::ServiceServer serviceGas = n.advertiseService("odor_value", get_gas_value_srv);
    ros::ServiceServer serviceWind = n.advertiseService("wind_value", get_wind_value_srv);

    //Init variables        
    init_all_simulation_instances();
    ros::Time time_last_loaded_file = ros::Time::now();
    srand(time(NULL));// initialize random seed

    //Init Markers for RVIZ visualization
    mkr_gas_points.header.frame_id = "/map";
    mkr_gas_points.header.stamp = ros::Time::now();
    mkr_gas_points.ns = "Gas_Dispersion";
    mkr_gas_points.action = visualization_msgs::Marker::ADD;
    mkr_gas_points.type = visualization_msgs::Marker::POINTS;   //Marker type
    mkr_gas_points.id = 0;                                      //One marker with multiple points.
    mkr_gas_points.scale.x = 0.025;
    mkr_gas_points.scale.y = 0.025;
    mkr_gas_points.scale.z = 0.025;
    mkr_gas_points.pose.orientation.w = 1.0;


    // Loop	
    ros::Rate r(100); //Set max rate at 100Hz (for handling services - Top Speed!!)
    int iteration_counter = initial_iteration;
    while (ros::ok())
    {        
        if( (ros::Time::now() - time_last_loaded_file).toSec() >= 1/player_freq )
        {
            if (verbose)
                ROS_INFO("[Player] Playing simulation iteration %i", iteration_counter);
            //Read Gas and Wind data from log_files
            load_all_data_from_logfiles(iteration_counter); //On the first time, we configure gas type, source pos, etc.
            display_current_gas_distribution();    //Rviz visualization
            iteration_counter++;

            //Looping?
            if (allow_looping)
            {
               if (iteration_counter >= loop_to_iteration)
               {
                   iteration_counter = loop_from_iteration;
                   if (verbose)
                       ROS_INFO("[Player] Looping");
               }
            }
            time_last_loaded_file = ros::Time::now();
        }

        //Attend service request at max rate!
        //This allows sensors to have higher sampling rates than the simulation update
        ros::spinOnce();
        r.sleep();
    }
}


//Load Node parameters
void loadNodeParameters(ros::NodeHandle private_nh)
{
    //player_freq
    private_nh.param<bool>("verbose", verbose, false);
    
    //player_freq
    private_nh.param<double>("player_freq", player_freq, 1);  //Hz

    //Number of simulators to load (For simulating multiple gases and multiple sources)
    private_nh.param<int>("num_simulators", num_simulators, 1);

    if (verbose)
    {
        ROS_INFO("[Player] player_freq %.2f", player_freq);
        ROS_INFO("[Player] num_simulators:  %i", num_simulators);
    }


    //FilePath for simulated data
    simulation_data.resize(num_simulators);
    for (int i=0;i<num_simulators; i++)
    {
        //Get location of simulation data for instance (i)
        std::string paramName = boost::str( boost::format("simulation_data_%i") % i);
        private_nh.param<std::string>(paramName.c_str(),simulation_data[i], "");
        if (verbose)
            ROS_INFO("[Player] simulation_data_%i:  %s", i, simulation_data[i].c_str());
    }
    
    // Initial iteration
    private_nh.param<int>("initial_iteration", initial_iteration, 1);
    // Loop
    private_nh.param<bool>("allow_looping", allow_looping, false);
    private_nh.param<int>("loop_from_iteration", loop_from_iteration, 1);
    private_nh.param<int>("loop_to_iteration", loop_to_iteration, 1);
    
}


//Init
void init_all_simulation_instances()
{
    ROS_INFO("[Player] Initializing %i instances",num_simulators);

    // At least one instance is needed which loads the wind field data!
    sim_obj so(simulation_data[0], true);
    player_instances.push_back(so);

    //Create other instances, but do not save wind information! It is the same for all instances
    for (int i=1;i<num_simulators;i++)
    {
        sim_obj so(simulation_data[i], false);
        player_instances.push_back(so);
    }

    //Set size for service responses
    srv_response_gas_types.resize(num_simulators);
    srv_response_gas_concs.resize(num_simulators);
}


//Load new Iteration of the Gas&Wind State on the 3d environment
void load_all_data_from_logfiles(int sim_iteration)
{    
    //Load corresponding data for each instance (i.e for every gas source)
    for (int i=0;i<num_simulators;i++)
    {
        if (verbose)
            ROS_INFO("[Player] Loading new data to instance %i (iteration %i)",i,sim_iteration);
        player_instances[i].load_data_from_logfile(sim_iteration);
    }
}


//Display in RVIZ the gas distribution
void display_current_gas_distribution()
{
    //Remove previous data points
    mkr_gas_points.points.clear();
    mkr_gas_points.colors.clear();
    for (int i=0;i<num_simulators;i++)
    {
        player_instances[i].get_concentration_as_markers(mkr_gas_points);
    }
    //Display particles
    marker_pub.publish(mkr_gas_points);
}

//==================================== SIM_OBJ ==============================//


// Constructor
sim_obj::sim_obj(std::string filepath, bool load_wind_info)
{
    gas_type = "unknown";
    simulation_filename = filepath;
    environment_cells_x = environment_cells_y = environment_cells_z = 0;
    environment_cell_size = 0.0; //m
    source_pos_x = source_pos_y = source_pos_z = 0.0; //m
    load_wind_data = load_wind_info;
    first_reading = true;
    filament_log=false;
}

sim_obj::~sim_obj(){}


void sim_obj::read_concentration_line(std::string line){
    size_t pos;
    double conc, u, v, w;
    int x, y, z;
    //A line has the format x y z conc u v w
    pos = line.find(" ");
    x = atoi(line.substr(0, pos).c_str());
    line.erase(0, pos + 1);

    pos = line.find(" ");
    y = atoi(line.substr(0, pos).c_str());
    line.erase(0, pos + 1);

    pos = line.find(" ");
    z = atoi(line.substr(0, pos).c_str());
    line.erase(0, pos + 1);

    pos = line.find(" ");
    conc = atof(line.substr(0, pos).c_str());
    line.erase(0, pos + 1);

    pos = line.find(" ");
    u = atof(line.substr(0, pos).c_str());
    line.erase(0, pos + 1);

    pos = line.find(" ");
    v = atof(line.substr(0, pos).c_str());
    w = atof(line.substr(pos + 1).c_str());

    //Save data to internal storage
    C[indexFrom3D(x,y,z)] = conc / 1000;
    if (load_wind_data)
    {
        U[indexFrom3D(x,y,z)] = u / 1000;
        V[indexFrom3D(x,y,z)] = v / 1000;
        W[indexFrom3D(x,y,z)] = w / 1000;
    }
}

void sim_obj::read_headers(std::stringstream &inbuf, std::string &line){
    std::getline(inbuf, line); 
    //Line 1 (min values of environment)
    size_t pos = line.find(" ");
    line.erase(0, pos + 1);
    pos = line.find(" ");
    env_min_x = atof(line.substr(0, pos).c_str());
    line.erase(0, pos + 1);
    pos = line.find(" ");
    env_min_y = atof(line.substr(0, pos).c_str());
    env_min_z = atof(line.substr(pos + 1).c_str());

    std::getline(inbuf, line); 
    //Line 2 (max values of environment)
    pos = line.find(" ");
    line.erase(0, pos + 1);
    pos = line.find(" ");
    env_max_x = atof(line.substr(0, pos).c_str());
    line.erase(0, pos + 1);
    pos = line.find(" ");
    env_max_y = atof(line.substr(0, pos).c_str());
    env_max_z = atof(line.substr(pos + 1).c_str());

    std::getline(inbuf, line); 
    //Get Number of cells (X,Y,Z)
    pos = line.find(" ");
    line.erase(0, pos + 1);

    pos = line.find(" ");
    environment_cells_x = atoi(line.substr(0, pos).c_str());
    line.erase(0, pos + 1);
    pos = line.find(" ");
    environment_cells_y = atoi(line.substr(0, pos).c_str());
    environment_cells_z = atoi(line.substr(pos + 1).c_str());

    std::getline(inbuf, line); 
    //Get Cell_size
    pos = line.find(" ");
    line.erase(0, pos + 1);

    pos = line.find(" ");
    environment_cell_size = atof(line.substr(0, pos).c_str());

    std::getline(inbuf, line); 
    //Get GasSourceLocation
    pos = line.find(" ");
    line.erase(0, pos + 1);
    pos = line.find(" ");
    source_pos_x = atof(line.substr(0, pos).c_str());
    line.erase(0, pos + 1);

    pos = line.find(" ");
    source_pos_y = atof(line.substr(0, pos).c_str());
    source_pos_z = atof(line.substr(pos + 1).c_str());

    std::getline(inbuf, line); 
    //Get Gas_Type
    pos = line.find(" ");
    gas_type = line.substr(pos + 1);
    //Configure instances
    configure_environment();

    std::getline(inbuf, line); 
    std::getline(inbuf, line); 
    std::getline(inbuf, line);
}

//Load a new file with Gas+Wind data
void sim_obj::load_data_from_logfile(int sim_iteration)
{
    std::string filename = boost::str( boost::format("%s/iteration_%i") % simulation_filename.c_str() % sim_iteration);
    FILE* fileCheck;
    if ((fileCheck =fopen(filename.c_str(),"rb"))==NULL){
        ROS_ERROR("File %s does not exist\n", filename.c_str());
        return;
    }
    fclose(fileCheck);

    std::ifstream infile(filename, std::ios_base::binary);
    boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;
    inbuf.push(boost::iostreams::zlib_decompressor());
    inbuf.push(infile);

    std::stringstream decompressed;
    boost::iostreams::copy(inbuf,decompressed);
    
    //if the file starts with a 1, the contents are in binary
    int check=0;
    decompressed.read((char*) &check, sizeof(int));
    if (check==1){
        filament_log=true;
        load_binary_file(decompressed);
    }
    else
        load_ascii_file(decompressed);
}

void sim_obj::load_ascii_file(std::stringstream& decompressed){
    std::string line;

    size_t pos;
    double conc, u, v, w;
    int x, y, z;

    if (first_reading)
    {
        read_headers(decompressed, line);
        first_reading=false;
    }
    else{
        //if already initialized, skip the header
        for(int i=0; i<8; i++){
            std::getline(decompressed, line);
        }
    }

    do
    {
        read_concentration_line(line);
    }while (std::getline(decompressed, line));

}

void sim_obj::load_binary_file(std::stringstream& decompressed){
    
    if(first_reading){
        double bufferD [5];
        decompressed.read((char*) &env_min_x, sizeof(double));
        decompressed.read((char*) &env_min_y, sizeof(double));
        decompressed.read((char*) &env_min_z, sizeof(double));

        decompressed.read((char*) &env_max_x, sizeof(double));
        decompressed.read((char*) &env_max_y, sizeof(double));
        decompressed.read((char*) &env_max_z, sizeof(double));

        decompressed.read((char*) &environment_cells_x, sizeof(int));
        decompressed.read((char*) &environment_cells_y, sizeof(int));
        decompressed.read((char*) &environment_cells_z, sizeof(int));

        decompressed.read((char*) &environment_cell_size, sizeof(double));
        
        decompressed.read((char*) &bufferD, 5*sizeof(double));
        int gt;
        decompressed.read((char*) &gt, sizeof(int));
        gas_type=gasTypesByCode[gt];

        decompressed.read((char*) &total_moles_in_filament, sizeof(double));
        decompressed.read((char*) &num_moles_all_gases_in_cm3, sizeof(double));

        configure_environment();
        first_reading=false;
    }else{
        //skip headers
        decompressed.seekg(14*sizeof(double) + 5*sizeof(int));
    }

    int wind_index;
    decompressed.read((char*) &wind_index, sizeof(int));

    activeFilaments.clear();
    int filament_index;
    double x, y, z, stdDev;
    while(decompressed.peek()!=EOF){

        decompressed.read((char*) &filament_index, sizeof(int));
        decompressed.read((char*) &x, sizeof(double));
        decompressed.read((char*) &y, sizeof(double));
        decompressed.read((char*) &z, sizeof(double));
        decompressed.read((char*) &stdDev, sizeof(double));

        std::pair<int, Vec4> pair(filament_index, Vec4(x, y, z, stdDev));
        activeFilaments.insert(pair);
    }

    load_wind_file(wind_index);

}

void sim_obj::load_wind_file(int wind_index){
    if(wind_index==last_wind_idx)
        return;
    last_wind_idx=wind_index;

    std::ifstream infile(boost::str( boost::format("%s/wind/wind_iteration_%i") % simulation_filename.c_str() % wind_index), std::ios_base::binary);
    infile.read((char*) U.data(), sizeof(double)* U.size());
    infile.read((char*) V.data(), sizeof(double)* U.size());
    infile.read((char*) W.data(), sizeof(double)* U.size());
    
}

//Get Gas concentration at lcoation (x,y,z)
void sim_obj::get_gas_concentration(float x, float y, float z, std::string &gas_name, double &gas_conc)
{
    if(filament_log){
        gas_conc=0;
        for(auto it = activeFilaments.begin(); it!=activeFilaments.end(); it++){
            Vec4 fil = it->second;
            double dist = sqrt((x-fil.x)*(x-fil.x) + (y-fil.y)*(y-fil.y) + (z-fil.z)*(z-fil.z) );

            if(dist < fil.w*5/100){
                gas_conc += concentration_from_filament(x, y, z, fil);
            }
        }
    }else{
        //Get cell idx from point location
        int xx,yy,zz;
        xx = (int)ceil((x - env_min_x)/environment_cell_size);
        yy = (int)ceil((y - env_min_y)/environment_cell_size);
        zz = (int)ceil((z - env_min_z)/environment_cell_size);
        //Get gas concentration from that cell
        gas_conc = C[indexFrom3D(xx,yy,zz)];
    }

    gas_name = gas_type;
}

double sim_obj::concentration_from_filament(float x, float y, float z, Vec4 filament){
    //calculate how much gas concentration does one filament contribute to the queried location
    double sigma = filament.w;
    double distance_cm = 100 * sqrt( pow(x-filament.x,2) + pow(y-filament.y,2) + pow(z-filament.z,2) );

    double num_moles_target_cm3 = (total_moles_in_filament /
        (sqrt(8*pow(M_PI,3)) * pow(sigma,3) )) * exp( -pow(distance_cm,2)/(2*pow(sigma,2)) );

    double ppm = num_moles_target_cm3/num_moles_all_gases_in_cm3 * 1000000; //parts of target gas per million

    return ppm;
}

//Get Wind concentration at lcoation (x,y,z)
void sim_obj::get_wind_value(float x, float y, float z, double &u, double &v, double &w)
{
    if (load_wind_data)
    {
        //Get cell idx from point location
        int xx,yy,zz;
        xx = (int)ceil((x - env_min_x)/environment_cell_size);
        yy = (int)ceil((y - env_min_y)/environment_cell_size);
        zz = (int)ceil((z - env_min_z)/environment_cell_size);

        //Set wind vectors from that cell
        u = U[indexFrom3D(xx,yy,zz)];
        v = V[indexFrom3D(xx,yy,zz)];
        w = W[indexFrom3D(xx,yy,zz)];
    }
    else
    {
        if (verbose)
            ROS_WARN("[Player] Request to provide Wind information when No Wind data is available!!");
    }
}


//Init instances (for running multiple simulations)
void sim_obj::configure_environment()
{
    //ROS_INFO("Configuring Enviroment");
    //ROS_INFO("\t\t Dimension: [%i,%i,%i] cells", environment_cells_x, environment_cells_y, environment_cells_z);
    //ROS_INFO("\t\t Gas_Type: %s", gas_type.c_str());
    //ROS_INFO("\t\t FilePath: %s", simulation_filename.c_str());
    //ROS_INFO("\t\t Source at location: [%.4f,%.4f,%.4f][m] ", source_pos_x, source_pos_y, source_pos_z);
    //ROS_INFO("\t\t Loading Wind Data: %i", load_wind_data);


    //Resize Gas Concentration container
    C.resize(environment_cells_x * environment_cells_y * environment_cells_z);
    

    //Resize Wind info container (if necessary)
    if (load_wind_data)
    {

        U.resize(environment_cells_x * environment_cells_y * environment_cells_z);
        V.resize(environment_cells_x * environment_cells_y * environment_cells_z);
        W.resize(environment_cells_x * environment_cells_y * environment_cells_z); 
    }
}


void sim_obj::get_concentration_as_markers(visualization_msgs::Marker &mkr_points)
{
    if(!filament_log){
        //For every cell, generate as much "marker points" as [ppm]
        for (int i=0;i<environment_cells_x;i++)
        {
            for (int j=0;j<environment_cells_y;j++)
            {
                for (int k=0;k<environment_cells_z;k++)
                {
                    geometry_msgs::Point p; //Location of point
                    std_msgs::ColorRGBA color;  //Color of point

                    double gas_value = C[indexFrom3D(i,j,k)];

                    for (int N=0;N<(int)round(gas_value/2);N++)
                    {
                        //Set point position (corner of the cell + random)
                        p.x = env_min_x + (i+0.5)*environment_cell_size + ((rand()%100)/100.0f)*environment_cell_size;
                        p.y = env_min_y + (j+0.5)*environment_cell_size + ((rand()%100)/100.0f)*environment_cell_size;
                        p.z = env_min_z + (k+0.5)*environment_cell_size + ((rand()%100)/100.0f)*environment_cell_size;

                        //Set color of particle according to gas type
                        color.a = 1.0;
                        if (!strcmp(gas_type.c_str(),"ethanol"))
                        {
                            color.r=0.2; color.g=0.9; color.b=0;
                        }
                        else if (!strcmp(gas_type.c_str(),"methane"))
                        {
                            color.r=0.9; color.g=0.1; color.b=0.1;
                        }
                        else if (!strcmp(gas_type.c_str(),"hydrogen"))
                        {
                            color.r=0.2; color.g=0.1; color.b=0.9;
                        }
                        else if (!strcmp(gas_type.c_str(),"propanol"))
                        {
                            color.r=0.8; color.g=0.8; color.b=0;
                        }
                        else if (!strcmp(gas_type.c_str(),"chlorine"))
                        {
                            color.r=0.8; color.g=0; color.b=0.8;
                        }
                        else if (!strcmp(gas_type.c_str(),"flurorine"))
                        {
                            color.r=0.0; color.g=0.8; color.b=0.8;
                        }
                        else if (!strcmp(gas_type.c_str(),"acetone"))
                        {
                            color.r=0.9; color.g=0.2; color.b=0.2;
                        }
                        else if (!strcmp(gas_type.c_str(),"neon"))
                        {
                            color.r=0.9; color.g=0; color.b=0;
                        }
                        else if (!strcmp(gas_type.c_str(),"helium"))
                        {
                            color.r=0.9; color.g=0; color.b=0;
                        }
                        else if (!strcmp(gas_type.c_str(),"hot_air"))
                        {
                            color.r=0.9; color.g=0; color.b=0;
                        }
                        else
                        {
                            ROS_INFO("[player] Setting Defatul Color");
                            color.r = 0.9; color.g = 0;color.b = 0;
                        }

                        //Add particle marker
                        mkr_points.points.push_back(p);
                        mkr_points.colors.push_back(color);
                    }

                }
            }
        }

    }
    else{
        for(auto it = activeFilaments.begin(); it!=activeFilaments.end(); it++){
            geometry_msgs::Point p; //Location of point
            std_msgs::ColorRGBA color;  //Color of point

            Vec4 filament = it->second;
            for (int i=0; i<10; i++){
                p.x=(filament.x-filament.w/100)+((rand()%100)/100.0f)*filament.w/100*2;
                p.y=(filament.y-filament.w/100)+((rand()%100)/100.0f)*filament.w/100*2;
                p.z=(filament.z-filament.w/100)+((rand()%100)/100.0f)*filament.w/100*2;

                color.a=1;
                color.r=0;
                color.g=1;
                color.b=0;
                //Add particle marker
                mkr_points.points.push_back(p);
                mkr_points.colors.push_back(color);
            }
        }
    }
}

int sim_obj::indexFrom3D(int x, int y, int z){
	return x + y*environment_cells_x + z*environment_cells_x*environment_cells_y;
}

