/*---------------------------------------------------------------------------------------
 * MAIN Node for the simulation of gas dispersal using a Filament-based approach.
 * This node loads the wind field (usually from CFD simulation), and simulates over it
 * different filaments to spread gas particles.
 *
 * Each filament is composed of a fixed number of gas molecules (Q)
 * Each filament is determined by its center position and width.
 * The width of a filament increases over time (Turbulent and molecular difussion)
 * The position of a filament is updated with the wind.
 *
 * The gas concentration at a given point is the sum of the concentration of all filaments.
 *
 * Thus, the gas concentration at the source location is determined by the number of molecules/filament and the number of filaments.
 *
 * A log file is recorded for every snapshot (time-step) with information about the gas
 * concentration and wind vector for every cell (3D) of the environment.
 *
 * The node implements the filament-base gas dispersal simulation. At each time step, the puffs
 * of filaments are sequentially released at a source location. Each puff is composed of n filaments.
 * Filaments are affected by turbulence and molecular diffusion along its path while being transported
 * by advection with the wind. The 3-dimensional positions of these filaments are represented by the points
 * of the “visualization msgs/markers”. At each time step, “Dispersal_Simulation” node calculates or
 * determines the positions of n filaments. Gas plumes are simulated with or without acceleration.
 *
 * It is very time consuming, and currently it runs in just one thread (designed to run offline).
 *
 * TODO: Cambiar std::vector por std::list para los FILAMENTOS
 ---------------------------------------------------------------------------------------*/


#include "filament_simulator/filament_simulator.h"



CFilamentSimulator::CFilamentSimulator()
{
	//Read parameters
	loadNodeParameters();

	//Set Publishers and Subscribers
	//-------------------------------
	marker_pub = n.advertise<visualization_msgs::Marker>("filament_visualization", 1);

	//Init variables
	//-----------------
    sim_time = 0.0;                      //Start at time = 0(sec)
    current_wind_snapshot = 0;           //Start with wind_iter= 0;
    current_simulation_step = 0;         //Start with iter= 0;
	//last_saved_step = -1;
	last_saved_step = results_min_time/results_time_step;
    wind_notified = false;               //To warm the user (only once) that no more wind data is found!

	//Init the Simulator
	initSimulator();


	//Fluid Dynamics Eq
	/*/-----------------
	* Ideal gas equation:
	* PV = nRT
	* P is the pressure of the gas (atm)
	* V is the volume of the gas (cm^3)
    * n is the amount of substance of gas (mol) = m/M where m=mass of the gas [g] and M is the molar mass
	* R is the ideal, or universal, gas constant, equal to the product of the Boltzmann constant and the Avogadro constant. (82.057338 cm^3·atm/mol·k)
	* T is the temperature of the gas (kelvin)
	*/
	double R = 82.057338;												   //[cm³·atm/mol·K] Gas Constant
	filament_initial_vol = pow(6*filament_initial_std,3);				   //[cm³] -> We approximate the infinite volumen of the 3DGaussian as 6 sigmas.
	env_cell_vol = pow(cell_size*100,3);									//[cm³] Volumen of a cell
	filament_numMoles = (envPressure*filament_initial_vol)/(R*envTemperature);//[mol] Num of moles of Air in that volume
	env_cell_numMoles = (envPressure*env_cell_vol)/(R*envTemperature);		//[mol] Num of moles of Air in that volume


	//The moles of target_gas in a Filament are distributted following a 3D Gaussian
	//Given the ppm value at the center of the filament, we approximate the total number of gas moles in that filament.
	filament_numMoles_of_gas = ( sqrt(8*pow(3.14159,3))*pow(filament_initial_std,3) ) * filament_ppm_center/pow(10,6);   //[cm³]
	double numMoles_in_cm3 = envPressure/(R*envTemperature);   //[mol/cm³]
	filament_numMoles_of_gas = filament_numMoles_of_gas * numMoles_in_cm3; //[moles_target_gas/filament] This is a CTE parameter!!

	ROS_INFO("[filament] filament_initial_vol [cm3]: %f",filament_initial_vol);
	ROS_INFO("[filament] env_cell_vol [cm3]: %f",env_cell_vol);
	ROS_INFO("[filament] filament_numMoles [mol]: %E",filament_numMoles);
	ROS_INFO("[filament] env_cell_numMoles [mol]: %E",env_cell_numMoles);
	ROS_INFO("[filament] filament_numMoles_of_gas [mol]: %E",filament_numMoles_of_gas);

	// Molecular gas mass [g/mol]
	// SpecificGravity(Air) = 1 (as reference)
	// Specific gravity is the ratio of the density of a substance to the density of a reference substance; equivalently,
	// it is the ratio of the mass of a substance to the mass of a reference substance for the same given volume.
	SpecificGravity[0] = 1.0378;	  //ethanol   (heavier than air)
    SpecificGravity[1] = 0.5537;	  //methane   (lighter than air)
	SpecificGravity[2] = 0.0696;	  //hydrogen  (lighter than air)
	SpecificGravity[6] = 1.4529;	  //acetone   (heavier than air)

	//To be updated
	SpecificGravity[3] = 58.124;	 //propanol   //gases heavier then air
	SpecificGravity[4] = 70.906;	 //chlorine
	SpecificGravity[5] = 37.996;	 //fluorine
	SpecificGravity[7] = 20.179;	 //neon	   //gases lighter than air
	SpecificGravity[8] = 4.002602;   //helium
	SpecificGravity[9] = 26.966;	 //hot_air


	//Init visualization
	//-------------------
	filament_marker.header.frame_id = fixed_frame;
	filament_marker.ns = "filaments";
	filament_marker.action = visualization_msgs::Marker::ADD;
	filament_marker.id = 0;
	filament_marker.type = visualization_msgs::Marker::POINTS;
	filament_marker.color.a = 1;
	
	
}



CFilamentSimulator::~CFilamentSimulator()
{
}



void CFilamentSimulator::loadNodeParameters()
{
	ros::NodeHandle private_nh("~");

	//Simulation Time (sec)
	private_nh.param<double>("sim_time", max_sim_time, 20.0);

	//Time increment between Gas snapshots (sec)
	private_nh.param<double>("time_step", time_step, 1.0);

	//Number of iterations to carry on = max_sim_time/time_step
	numSteps = floor(max_sim_time/time_step);

	//Num of filaments/sec
	private_nh.param<int>("num_filaments_sec", numFilaments_sec, 100);
	numFilaments_step = floor(numFilaments_sec * time_step);
	total_number_filaments = numFilaments_step * numSteps;

	//Gas concentration at the filament center - 3D gaussian [ppm]
	private_nh.param<double>("ppm_filament_center", filament_ppm_center, 20);

	//[cm] Sigma of the filament at t=0-> 3DGaussian shape
	private_nh.param<double>("filament_initial_std", filament_initial_std, 1.5);

	//[cm²/s] Growth ratio of the filament_std
	private_nh.param<double>("filament_growth_gamma", filament_growth_gamma, 10.0);

	//[cm] Sigma of the white noise added on each iteration
	private_nh.param<double>("filament_noise_std", filament_noise_std, 0.1);

	//Gas Type ID
	private_nh.param<int>("gas_type", gasType, 1);

	// Environment temperature (necessary for molecules/cm3 -> ppm)
	private_nh.param<double>("temperature", envTemperature, 298.0);

	// Enviorment pressure (necessary for molecules/cm3 -> ppm)
	private_nh.param<double>("pressure", envPressure, 1.0);

	// Gas concentration units (0= molecules/cm3,  1=ppm)
	private_nh.param<int>("concentration_unit_choice", gasConc_unit, 1);



	//WIND DATA
	//----------
	//CFD wind files location
	private_nh.param<std::string>("wind_data", wind_files_location, "");

	//(sec) Time increment between Wind snapshots --> Determines when to load a new wind field
	private_nh.param<double>("wind_time_step", windTime_step, 1.0);




	//ENVIRONMENT
	//-----------
	// Occupancy gridmap 3D location
	private_nh.param<std::string>("occupancy3D_data", occupancy3D_data, "");

	//fixed frame (to disaply the gas particles on RVIZ)
	private_nh.param<std::string>("fixed_frame", fixed_frame, "/map");

	//Environment size (Num of cells): Set accordingly with the CFD wind fields
	//private_nh.param<int>("env_cells_x", env_cells_x, 10);
	//private_nh.param<int>("env_cells_y", env_cells_y, 10);
	//private_nh.param<int>("env_cells_z", env_cells_z, 10);

	//Environment Cell size
	//private_nh.param<double>("cell_size", cell_size, 1.0);

	//Source postion (x,y,z)
	private_nh.param<double>("source_position_x", gas_source_pos_x, 1.0);
	private_nh.param<double>("source_position_y", gas_source_pos_y, 1.0);
	private_nh.param<double>("source_position_z", gas_source_pos_z, 1.0);

	// Simulation results.
	private_nh.param<int>("save_results", save_results, 1);
	private_nh.param<std::string>("results_location", results_location, "");
    private_nh.param<double>("results_min_time", results_min_time, 0.0);
    private_nh.param<double>("results_time_step", results_time_step, 1.0);


	ROS_INFO("[filament] The data provided in the roslaunch file is:");
	ROS_INFO("[filament] Simulation Time	  %f(s)",sim_time);
	ROS_INFO("[filament] Gas Time Step:	   %f(s)",time_step);
	ROS_INFO("[filament] Num_steps:		   %d",numSteps);
	ROS_INFO("[filament] Number of filaments: %d",numFilaments_sec);
	ROS_INFO("[filament] PPM filament center  %f",filament_ppm_center);
	ROS_INFO("[filament] Gas type:			%d",gasType);
	ROS_INFO("[filament] Concentration unit:  %d",gasConc_unit);
	ROS_INFO("[filament] Wind_time_step:	  %f(s)", windTime_step);
	ROS_INFO("[filament] Fixed frame:		 %s",fixed_frame.c_str());
	ROS_INFO("[filament] Source position:	 (%f,%f,%f)",gas_source_pos_x, gas_source_pos_y, gas_source_pos_z);
}


void CFilamentSimulator::initSimulator()
{
	ROS_INFO("[filament] Initializing Simulator... Please Wait!");

	//1. Load Environment and Configure Matrices
	if (FILE *file = fopen(occupancy3D_data.c_str(), "r"))
	{
		//Files exist!, keep going!
		fclose(file);
		ROS_INFO("[filament] Loading 3D Occupancy GridMap");
		read_3D_file(occupancy3D_data, Env, true);
	}
	else
	{
		ROS_ERROR("[filament] File %s Does Not Exists!",occupancy3D_data.c_str());
	}


	//2. Load the first Wind snapshot from file (all 3 components U,V,W)
	read_wind_snapshot(current_simulation_step);

	//3. Initialize the filaments vector to its max value (to avoid increasing the size at runtime)
	ROS_INFO("[filament] Initializing Filaments");
	filaments.resize(total_number_filaments, CFilament(0.0, 0.0, 0.0, filament_initial_std));
}


//Resize a 3D Matrix compose of Vectors, This operation is only performed once!
void CFilamentSimulator::configure3DMatrix(std::vector< std::vector< std::vector<double> > > &A)
{
	A.resize(env_cells_x);
	for (int i = 0; i < env_cells_x; ++i)
	{
		A[i].resize(env_cells_y);
		for (int j = 0; j < env_cells_y; ++j)
		{
			A[i][j].resize(env_cells_z);
		}
	}
}


void CFilamentSimulator::read_wind_snapshot(int idx)
{

	//configure filenames to read
	std::string U_filemane = boost::str( boost::format("%s%i.csv_U") % wind_files_location % idx );
	std::string V_filemane = boost::str( boost::format("%s%i.csv_V") % wind_files_location % idx );
	std::string W_filemane = boost::str( boost::format("%s%i.csv_W") % wind_files_location % idx );

    ROS_INFO("Reading Wind Snapshot %s",U_filemane.c_str());

    //read data to 3D matrices
	if (FILE *file = fopen(U_filemane.c_str(), "r"))
	{
		//Files exist!, keep going!
		fclose(file);
		ROS_INFO("[filament] Loading Wind Snapshot %i", idx);

		read_3D_file(U_filemane, U, false);
		read_3D_file(V_filemane, V, false);
		read_3D_file(W_filemane, W, false);
	}
	else
	{
		//No more wind data. Keep current info.
		if (!wind_notified)
		{
			ROS_WARN("[filament] File %s Does Not Exists!",U_filemane.c_str());
			ROS_WARN("[filament] No more wind data available. Using last Wind snapshopt as SteadyState.");
			wind_notified = true;
		}
	}

	//Update the current idx
	current_wind_snapshot = idx;
}



void CFilamentSimulator::read_3D_file(std::string filename, std::vector< std::vector< std::vector<double> > > &A, bool hasHeader=false)
{
	//open file
	std::ifstream infile(filename.c_str());
	std::string line;
	int line_counter = 0;

	//If header -> read 4 Header lines & configure all matrices to given dimensions!
	if (hasHeader)
	{
		//Line 1 (min values of environment)
		std::getline(infile, line);
		line_counter++;
		size_t pos = line.find(" ");
		line.erase(0, pos+1);
		pos = line.find(" ");
		env_min_x = atof(line.substr(0, pos).c_str());
		line.erase(0, pos+1);
		pos = line.find(" ");
		env_min_y = atof(line.substr(0, pos).c_str());
		env_min_z = atof(line.substr(pos+1).c_str());


		//Line 2 (max values of environment)
		std::getline(infile, line);
		line_counter++;
		pos = line.find(" ");
		line.erase(0, pos+1);
		pos = line.find(" ");
		env_max_x = atof(line.substr(0, pos).c_str());
		line.erase(0, pos+1);
		pos = line.find(" ");
		env_max_y = atof(line.substr(0, pos).c_str());
		env_max_z = atof(line.substr(pos+1).c_str());

		//Line 3 (Num cells on eahc dimension)
		std::getline(infile, line);
		line_counter++;
		pos = line.find(" ");
		line.erase(0, pos+1);
		pos = line.find(" ");
		env_cells_x = atoi(line.substr(0, pos).c_str());
		line.erase(0, pos+1);
		pos = line.find(" ");
		env_cells_y = atof(line.substr(0, pos).c_str());
		env_cells_z = atof(line.substr(pos+1).c_str());

		//Line 4 cell_size (m)
		std::getline(infile, line);
		line_counter++;
		pos = line.find(" ");
		cell_size = atof(line.substr(pos+1).c_str());

		ROS_INFO("[filament] Env dimensions (%.2f,%.2f,%.2f) to (%.2f,%.2f,%.2f)",env_min_x, env_min_y, env_min_z, env_max_x, env_max_y, env_max_z );
		ROS_INFO("[filament] Env size in cells	 (%d,%d,%d) - with cell size %f [m]",env_cells_x,env_cells_y,env_cells_z, cell_size);

		//Reserve memory for the 3D matrices: U,V,W,C and Env, according to provided num_cells of the environment.
		//It also init them to 0.0 values
		configure3DMatrix(U);
		configure3DMatrix(V);
		configure3DMatrix(W);
		configure3DMatrix(C);
		configure3DMatrix(Env);
	}

    ROS_INFO("Reading File to (%lu,%lu,%lu)Matrix", A.size(), A[0].size(), A[0][0].size());

	//Read file line by line
	int x_idx = 0;
	int y_idx = 0;
	int z_idx = 0;

	while ( std::getline(infile, line) )
	{
		line_counter++;
		std::stringstream ss(line);
		if (z_idx >=env_cells_z)
		{
			ROS_ERROR("Trying to read:[%s]",line.c_str());
		}

		if (line == ";")
		{
			//New Z-layer
			z_idx++;
			x_idx = 0;
			y_idx = 0;
		}
		else
		{   //New line with constant x_idx and all the y_idx values
			while (!ss.fail())
			{
				double f;
				ss >> f;		//get one double value
				if (!ss.fail())
				{
					A[x_idx][y_idx][z_idx] = f;
					y_idx++;
				}
			}

			//Line has ended
			x_idx++;
			y_idx = 0;
		}
	}
    //End of file.
    ROS_INFO("End of File");
}



// Add new filaments. On each step add a total of "numFilaments_step"
void CFilamentSimulator::add_new_filaments(double radius_arround_source)
{
	for (int i=0; i<numFilaments_step; i++)
	{
		//Set position of new filament within the especified radius arround the gas source location
		double x = gas_source_pos_x + random_number(-1,1)*radius_arround_source;
		double y = gas_source_pos_y + random_number(-1,1)*radius_arround_source;
		double z = gas_source_pos_z + random_number(-1,1)*radius_arround_source;

		/*Instead of adding new filaments to the filaments vector on each iteration (push_back)
		  we had initially resized the filaments vector to the max number of filaments (numSteps*numFilaments_step)
		  Here we will "active" just the corresponding filaments for this step.*/
        filaments[current_simulation_step*numFilaments_step+i].activate_filament(x, y, z, sim_time);
	}
}


// Here we estimate the gas concentration on each cell of the 3D env
// based on the active filaments and their 3DGaussian shapes
// For that we employ Farrell's Concentration Eq
void CFilamentSimulator::update_gas_concentration_from_filaments()
{
	//First, set all cells to 0.0 gas concentration (clear previous state)
	for (size_t i=0; i<env_cells_x; i++)
	{
		for (size_t j=0; j<env_cells_y; j++)
		{
			for (size_t k=0; k<env_cells_z; k++)
			{
				C[i][j][k] = 0.0;
			}
		}
	}

	// We run over all the active filaments, and update the gas concentration of the cells that are close to them.
	// Ideally a filament spreads over the entire environment, but in practice since filaments are modeled as 3Dgaussians
	// We can stablish a cutt_off raduis of 3*sigma.
    // To avoid resolution problems, we evaluate each filament according to the minimum between:
    // the env_cell_size and filament_sigma. This way we ensure a filament is always well evaluated (not only one point).
	for (int fil_i=0; fil_i<current_simulation_step*numFilaments_step; fil_i++)
	{
		if (filaments[fil_i].valid)
		{
			double grid_size_m = std::min(cell_size, (filaments[fil_i].sigma/100));  //[m] grid size to evaluate the filament
					// Compute at which increments the Filament has to be evaluated.
					// If the sigma of the Filament is very big (i.e. the Filament is very flat), the use the world's cell_size.
					// If the Filament is very small (i.e in only spans one or few world cells), then use increments equal to sigma
					//  in order to have several evaluations fall in the same cell.

			int num_evaluations = ceil(6*(filaments[fil_i].sigma/100) / grid_size_m);
					// How many times the Filament has to be evaluated depends on the final grid_size_m.
					// The filament's grid size is multiplied by 6 because we evaluate it over +-3 sigma
					// If the filament is very small (i.e. grid_size_m = sigma), then the filament is evaluated only 6 times
					// If the filament is very big and spans several cells, then it has to be evaluated for each cell (which will be more than 6)


			// EVALUATE IN ALL THREE AXIS
			for (int i=0; i<=num_evaluations; i++)
			{
				for (int j=0; j<=num_evaluations; j++)
				{
					for (int k=0; k<=num_evaluations; k++)
					{
						//get point to evaluate [m]
						double x = (filaments[fil_i].pose_x - 3*(filaments[fil_i].sigma/100)) + i*grid_size_m;
						double y = (filaments[fil_i].pose_y - 3*(filaments[fil_i].sigma/100)) + j*grid_size_m;
						double z = (filaments[fil_i].pose_z - 3*(filaments[fil_i].sigma/100)) + k*grid_size_m;

						// Disntance from evaluated_point to filament_center (in [cm])
						double distance_cm = 100 * sqrt( pow(x-filaments[fil_i].pose_x,2) + pow(y-filaments[fil_i].pose_y,2) + pow(z-filaments[fil_i].pose_z,2) );

						// FARRELLS Eq.
                        //Evaluate the concentration of filament fil_i at given point (moles/cm³)
						double num_moles_cm3 = (filament_numMoles_of_gas / (sqrt(8*pow(3.14159,3)) * pow(filaments[fil_i].sigma,3) )) * exp( -pow(distance_cm,2)/(2*pow(filaments[fil_i].sigma,2)) );

						//Multiply for the volumen of the grid cell
						double num_moles = num_moles_cm3 * pow(grid_size_m*100,3);  //[moles]


						//Valid point? If either OUT of the environment, or through a wall, treat it as invalid
						bool path_is_obstructed =  check_environment_for_obstacle(filaments[fil_i].pose_x, filaments[fil_i].pose_y, filaments[fil_i].pose_z, x,y,z );

                        if (!path_is_obstructed)
                        {
                            if (path_is_obstructed)
                            {
                                //Point is not valid! Instead of ignoring it, add its concentration to the filament center location
                                //This avoids "loosing" gas concentration as filaments get close to obstacles (e.g. the floor)
                                x = filaments[fil_i].pose_x;
                                y = filaments[fil_i].pose_y;
                                z = filaments[fil_i].pose_z;
                            }

                            //Get 3D cell of the evaluated point
                            int x_idx = floor( (x-env_min_x)/cell_size );
                            int y_idx = floor( (y-env_min_y)/cell_size );
                            int z_idx = floor( (z-env_min_z)/cell_size );

                            //Accumulate concentration in corresponding env_cell
                            if (gasConc_unit==0)
                                C[x_idx][y_idx][z_idx] += num_moles;	//moles
                            else
                            {
                                double num_ppm = (num_moles/env_cell_numMoles)*pow(10,6);   //[ppm]
                                C[x_idx][y_idx][z_idx] += num_ppm;	//ppm
                                //ROS_INFO("ppm concentraton at [%i %i %i]: %f", x_idx,y_idx,z_idx,num_ppm);
                            }
                        }
					}
				}
			}
		}
	}

	//Update Gasconcentration markers
}



//Check if a given 3D pose falls in:
// 0 = free space
// 1 = obstacle, wall, or outside the environment
// 2 = outlet (usefull to disable filaments)
int CFilamentSimulator::check_pose_with_environment(double pose_x, double pose_y, double pose_z)
{
	//1.1 Check that pose is within the boundingbox environment
	if (pose_x<env_min_x || pose_x>env_max_x || pose_y<env_min_y || pose_y>env_max_y || pose_z<env_min_z || pose_z>env_max_z)
		return 1;

	//Get 3D cell of the point
	int x_idx = floor( (pose_x-env_min_x)/cell_size );
	int y_idx = floor( (pose_y-env_min_y)/cell_size );
	int z_idx = floor( (pose_z-env_min_z)/cell_size );

	if (x_idx >= env_cells_x || y_idx >= env_cells_y || z_idx >= env_cells_z)
		return 1;

	//1.2. Return cell occupancy (0=free, 1=obstacle, 2=outlet)
	return Env[x_idx][y_idx][z_idx];
}

bool CFilamentSimulator::check_environment_for_obstacle(double start_x, double start_y, double start_z,
													   double   end_x, double   end_y, double end_z)
{
	const bool PATH_OBSTRUCTED = true;
	const bool PATH_UNOBSTRUCTED = false;


	// Check whether one of the points is outside the valid environment or is not free
	if(check_pose_with_environment(start_x, start_y, start_z) != 0)   { return PATH_OBSTRUCTED; }
	if(check_pose_with_environment(  end_x, end_y  ,   end_z) != 0)   { return PATH_OBSTRUCTED; }


	// Calculate normal displacement vector
	double vector_x = end_x - start_x;
	double vector_y = end_y - start_y;
	double vector_z = end_z - start_z;
	double distance = sqrt(vector_x*vector_x + vector_y*vector_y + vector_z*vector_z);
	vector_x = vector_x/distance;
	vector_y = vector_y/distance;
	vector_z = vector_z/distance;


	// Traverse path
	int steps = ceil( distance / cell_size );	// Make sure no two iteration steps are separated more than 1 cell
	double increment = distance/steps;

	for(int i=1; i<steps-1; i++)
	{
		// Determine point in space to evaluate
		double pose_x = start_x + vector_x*increment*i;
		double pose_y = start_y + vector_y*increment*i;
		double pose_z = start_z + vector_z*increment*i;


		// Determine cell to evaluate (some cells might get evaluated twice due to the current code
		int x_idx = floor( (pose_x-env_min_x)/cell_size );
		int y_idx = floor( (pose_y-env_min_y)/cell_size );
		int z_idx = floor( (pose_z-env_min_z)/cell_size );


		// Check if the cell is occupied
		if(Env[x_idx][y_idx][z_idx] != 0) { return PATH_OBSTRUCTED; }
	}

	// Direct line of sight confirmed!
	return PATH_UNOBSTRUCTED;
}



//Update the filaments location in the 3D environment
// According to Farrell Filament model, a filament is afected by three components of the wind flow.
// 1. Va (large scale wind) -> Advection (Va) -> Movement of a filament as a whole by wind) -> from CFD
// 2. Vm (middle scale wind)-> Movement of the filament with respect the center of the "plume" -> modeled as white noise
// 3. Vd (small scale wind) -> Difussion or change of the filament shape (growth with time)
// We also consider Gravity and Bouyant Forces given the gas molecular mass
void CFilamentSimulator::update_filaments_location()
{
	//Estimte filament acceleration due to gravity & Bouyant force (for the given gas_type):
	double g = 9.8;
	double specific_gravity_air = 1; //[dimensionless]
	double accel = g * ( specific_gravity_air - SpecificGravity[gasType] ) / SpecificGravity[gasType];
	double newpos_x, newpos_y, newpos_z, noise_std;

	//Update the location of all active filaments
	//ROS_INFO("[filament] Updating %i filaments of %lu",current_simulation_step*numFilaments_step, filaments.size());

	for (int i=0; i<current_simulation_step*numFilaments_step; i++)
	{
		if (filaments[i].valid)
		{
			try
			{
				//Get 3D cell of the filament center
				int x_idx = floor( (filaments[i].pose_x-env_min_x)/cell_size );
				int y_idx = floor( (filaments[i].pose_y-env_min_y)/cell_size );
				int z_idx = floor( (filaments[i].pose_z-env_min_z)/cell_size );

				//1. Simulate Advection (Va)
				//   Large scale wind-eddies -> Movement of a filament as a whole by wind
				//------------------------------------------------------------------------
				newpos_x = filaments[i].pose_x + U[x_idx][y_idx][z_idx] * time_step;
				newpos_y = filaments[i].pose_y + V[x_idx][y_idx][z_idx] * time_step;
				newpos_z = filaments[i].pose_z + W[x_idx][y_idx][z_idx] * time_step;

				//Check filament location
				int valid_location = check_pose_with_environment(newpos_x, newpos_y, newpos_z);
				switch (valid_location)
				{
				case 0:
					//Free and valid location... update filament position
					filaments[i].pose_x = newpos_x;
					filaments[i].pose_y = newpos_y;
					filaments[i].pose_z = newpos_z;
					break;
				case 2:
					//The location corresponds to an outlet! Delete filament!
					filaments[i].valid = false;
					break;
				default:
					//The location falls in an obstacle -> Illegal movement (Do not apply advection)
					break;
				}

				//2. Simulate Gravity & Bouyant Force
				//------------------------------------
                //OLD approach: using accelerations (pure gas)
                //newpos_z = filaments[i].pose_z + 0.5*accel*pow(time_step,2);

                //Approximation from "Terminal Velocity of a Bubble Rise in a Liquid Column", World Academy of Science, Engineering and Technology 28 2007
                double ro_air = 1.205;     //[kg/m³] density of air
                double mu = 19*pow(10,-6);    //[kg/s·m] dynamic viscosity of air
                double terminal_buoyancy_velocity = (g * (1-SpecificGravity[gasType])*ro_air * filament_ppm_center*pow(10,-6) ) / (18* mu);
                newpos_z = filaments[i].pose_z + terminal_buoyancy_velocity*time_step;


				//Check filament location
				if (check_pose_with_environment(filaments[i].pose_x, filaments[i].pose_y, newpos_z ) == 0)
					filaments[i].pose_z = newpos_z;


				//3. Add some variability (stochastic process)
				//   Vm (middle scale wind)-> Movement of the filament with respect the center of the "plume" -> modeled as Gaussian white noise
				//----------------------------------------------------------------
				noise_std = filament_noise_std;// * time_step;
				static RandomGenerator rng(static_cast<unsigned> (time(0)));
				NormalDistribution gaussian_dist(0.0,noise_std);
				GaussianGenerator generator(rng, gaussian_dist);

				newpos_x = filaments[i].pose_x + generator();
				newpos_y = filaments[i].pose_y + generator();
				newpos_z = filaments[i].pose_z + generator();

				//Check filament location
				if (check_pose_with_environment(newpos_x, newpos_y, newpos_z ) == 0)
				{
					filaments[i].pose_x = newpos_x;
					filaments[i].pose_y = newpos_y;
					filaments[i].pose_z = newpos_z;
				}

				//4. Filament growth with time (this affects the posterior estimation of gas concentration at each cell)
				//   Vd (small scale wind eddies) -> Difussion or change of the filament shape (growth with time)
				//   R = sigma of a 3D gaussian -> Increasing sigma with time
				//------------------------------------------------------------------------
                filaments[i].sigma = sqrt(pow(filament_initial_std,2) + filament_growth_gamma*(sim_time-filaments[i].birth_time));

			}catch(...)
			{
				ROS_ERROR("Exception Updating Filaments!");
				return;
			}
		}
	}
}


void CFilamentSimulator::publish_markers()
{
	//1. Clean old markers
	filament_marker.points.clear();
	filament_marker.colors.clear();
	filament_marker.header.stamp = ros::Time::now();
	filament_marker.pose.orientation.w = 1.0;

	//width of points: scale.x is point width, scale.y is point height
	filament_marker.scale.x = cell_size/4;
	filament_marker.scale.y = cell_size/4;
	filament_marker.scale.z = cell_size/4;

	//2. Add a marker for each filament!
	for (int i=0; i<current_simulation_step*numFilaments_step; i++)
	{
		geometry_msgs::Point point;
		std_msgs::ColorRGBA color;

		//Set filament pose
		point.x = filaments[i].pose_x;
		point.y = filaments[i].pose_y;
		point.z = filaments[i].pose_z;

		//Set filament color
		color.a = 1;
		if (filaments[i].valid)
		{
			color.r = 0;
			color.g = 0;
			color.b = 1;
		}
		else
		{
			color.r = 1;
			color.g = 0;
			color.b = 0;
		}

		//Add marker
		filament_marker.points.push_back(point);
		filament_marker.colors.push_back(color);
	}

	//Publish marker of the filaments
	marker_pub.publish(filament_marker);
}


double CFilamentSimulator::random_number(double min_val, double max_val)
{
	double n = (double)(rand() % 100);  //int random number [0, 100)
	n = n/100.0f;					   //random number [0, 1)
	n = n * (max_val - min_val);		//random number [0, max-min)
	n = n+ min_val;					 //random number [min, max)
	return n;
}


//Saves current Wind + GasConcentration to file
// These files will be later used in the "player" node.
void CFilamentSimulator::save_state_to_file()
{
	//Configure file name for saving the current snapshot
	FILE *out;
	std::string out_filemane = boost::str( boost::format("%s/FilamentSimulation_gasType_%i_sourcePosition_%.2f_%.2f_%.2f_iteration_%i") % results_location % gasType % gas_source_pos_x % gas_source_pos_y % gas_source_pos_z % last_saved_step);

	//open file
	//---------
	out = fopen( out_filemane.c_str(), "w" );
	if( out == NULL )
		ROS_ERROR("Error in opening the final output files.\n");

	//Write header
	//--------------
	fprintf(out, "env_min(m) %.4f %.4f %.4f\n",env_min_x, env_min_y, env_min_z);
	fprintf(out, "env_max(m) %.4f %.4f %.4f\n",env_max_x, env_max_y, env_max_z);
	fprintf(out, "NumCells_XYZ %i %i %i\n",env_cells_x,env_cells_y,env_cells_z);
	fprintf(out, "CellSizes_XYZ[m] %.4f %.4f %.4f\n",cell_size, cell_size, cell_size);
	fprintf(out, "GasSourceLocation_XYZ[m] %.4f %.4f %.4f\n",gas_source_pos_x, gas_source_pos_y, gas_source_pos_z);
	std::string gas_type_str;
	switch (gasType)
	{
		case 0: gas_type_str = "ethanol"; break;
		case 1: gas_type_str = "methane"; break;
		case 2: gas_type_str = "hydrogen"; break;
		case 3: gas_type_str = "propanol"; break;
		case 4: gas_type_str = "chlorine"; break;
		case 5: gas_type_str = "flurorine"; break;
		case 6: gas_type_str = "acetone"; break;
		case 7: gas_type_str = "neon"; break;
		case 8: gas_type_str = "helium"; break;
		case 9: gas_type_str = "hot_air"; break;
		default: gas_type_str = "ethanol";
	}
	fprintf(out, "GasType %s\n",gas_type_str.c_str());

	std::string gas_units_str;
	if (gasConc_unit == 0)
		gas_units_str = "moles";
	else if (gasConc_unit == 1)
		gas_units_str =  "ppm";
	else
		gas_units_str =  "ppm";
	fprintf(out, "Cell_x\t Cell_y\t Cell_z\t Gas_conc[%s]\t Wind_u[m/s]\t Wind_v[m/s]\t Wind_w[m/s]\n", gas_units_str.c_str());


	// Save to file the Gas concentration and wind vectors of every cell of the environment.
	//-------------------------------------------------------------------------------------
	for (int x = 0; x < env_cells_x; x++)
	{
		for (int y = 0; y < env_cells_y; y++)
		{
			for (int z = 0; z < env_cells_z; z++)
			{
				//Save to file! -> "Cell_x Cell_y Cell_z, Gas_conc U V W"
				fprintf(out, "%i %i %i %f %f %f %f\n",x, y, z, C[x][y][z], U[x][y][z], V[x][y][z], W[x][y][z]);
			}
		}
	}

	//Close file (log file)
	fclose(out);
	last_saved_step++;
}

//================================
//
//			M   A   I   N
//
//================================
int main(int argc, char **argv)
{
	// Init ROS-NODE
	ros::init(argc, argv, "new_filament_simulator");

	//Create simulator obj and initialize it
	CFilamentSimulator sim;


	// Initiate Random Number generator with current time
	srand(time(NULL));

	//--------------
	// LOOP
	//--------------	
	ros::Rate r(100);   //Go as faster as you can!
	while (ros::ok() && (sim.current_simulation_step<sim.numSteps) )
	{
		//ROS_INFO("[filament] Simulating step %i (sim_time = %.2f)", sim.current_simulation_step, sim.sim_time);

		//0. Load wind snapshot (if necessary and availabe)
		if ( floor(sim.sim_time/sim.windTime_step) != sim.current_wind_snapshot)
			sim.read_wind_snapshot(floor(sim.sim_time/sim.windTime_step));

		//1. Create new filaments close to the source location
		//   On each iteration num_filaments (See params) are created
		sim.add_new_filaments(sim.cell_size);

		//2. Update Gas Concentration field		
		sim.update_gas_concentration_from_filaments();

		//3. Publish markers for RVIZ
		sim.publish_markers();

		//4. Update filament locations
		sim.update_filaments_location();

		//5. Save data (if necessary)
        if ( (sim.save_results==1) && (sim.sim_time>=sim.results_min_time) )
        {
            if ( floor(sim.sim_time/sim.results_time_step) != sim.last_saved_step )
			    sim.save_state_to_file();
	    }

		//4. Update Simulation state
		sim.sim_time = sim.sim_time + sim.time_step;	//sec
		sim.current_simulation_step++;

		r.sleep();
		ros::spinOnce();
	}
}
