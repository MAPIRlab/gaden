#include "preprocessing/preprocessing.h"

void printEnv(std::string filename, std::vector<std::vector<std::vector<int> > > env, int scale)
{
    std::ofstream outfile(filename.c_str());
    if (filename.find(".pgm") != std::string::npos)
    {
        outfile << "P2\n"
                << scale *  env[0].size() << " " << scale * env.size() << "\n" <<"1\n";
        //things are repeated to scale them up (the image is too small!)
        for (int row = env.size()-1; row >= 0; row--)
        {
            for (int j = 0; j < scale; j++)
            {
                for (int col = 0; col <env[0].size() ; col++)
                {
                    for (int i = 0; i < scale; i++)
                    {
                        outfile << (env[row][col][0] == 3 ? 1 : 0) << " ";
                    }
                }
                outfile << "\n";
            }
        }
    }
    else
    {
        outfile <<  "#env_min(m) " << env_min_x << " " << env_min_y << " " << env_min_z << "\n";
        outfile <<  "#env_max(m) " << env_max_x << " " << env_max_y << " " << env_max_z << "\n";
        outfile <<  "#num_cells " << env[0].size() << " " << env.size() << " " << env[0][0].size() << "\n";
        outfile <<  "#cell_size(m) " << cell_size << "\n";
        //things are repeated to scale them up (the image is too small!)
        for (int height = 0; height < env[0][0].size(); height++)
        {
            for (int col = 0; col <env[0].size(); col++)
            {
                for (int j = 0; j < scale; j++)
                {
                    for (int row = 0; row <env.size(); row++)
                    {
                        for (int i = 0; i < scale; i++)
                        {
                            outfile << (env[row][col][height]==0?1:
                                            (env[row][col][height]==3?0:
                                                env[row][col][height])) 
                                    << " ";
                        }
                    }
                    outfile << "\n";
                }
            }
            outfile << ";\n";
        }
    }
}
void printWind(std::vector<std::vector<std::vector<double> > > U,
                std::vector<std::vector<std::vector<double> > > V,
                std::vector<std::vector<std::vector<double> > > W, std::string filename){
    
    std::ofstream fileU(boost::str(boost::format("%s_U") % filename).c_str());
    std::ofstream fileV(boost::str(boost::format("%s_V") % filename).c_str());
    std::ofstream fileW(boost::str(boost::format("%s_W") % filename).c_str());
    for (int height = 0; height < U[0][0].size(); height++)
    {
        for (int col = 0; col < U.size(); col++)
        {
            for (int row = 0; row < U[0].size(); row++)
            {
                fileU << U[col][row][height] << " ";
                fileV << V[col][row][height] << " ";
                fileW << W[col][row][height] << " ";
            }
            fileU << "\n";
            fileV << "\n";
            fileW << "\n";
        }
        fileU << ";\n";
        fileV << ";\n";
        fileW << ";\n";
    }
}

double min_val(double x, double y, double z) {

    double min = 99999;

    if (x < min)
        min=x;
    if (y < min)
        min=y;
    if(z < min)
        min=z;

    return min;
}
double max_val(double x, double y, double z) {

    double max = -99999;

    if (x > max)
        max=x;
    if (y > max)
        max=y;
    if(z > max)
        max=z;

    return max;
}
bool eq(double x, double y){
    return std::abs(x-y)<(cell_size/10);
}
std::vector<Eigen::Vector3d> cubePoints(const Eigen::Vector3d &query_point){
    std::vector<Eigen::Vector3d> points;
    points.push_back(query_point);
    points.push_back(Eigen::Vector3d(query_point(0)-cell_size/2,
                                        query_point(1)-cell_size/2,
                                        query_point(2)-cell_size/2));
    points.push_back(Eigen::Vector3d(query_point(0)+cell_size/2,
                                        query_point(1)+cell_size/2,
                                        query_point(2)+cell_size/2));
    points.push_back(Eigen::Vector3d(query_point(0)+cell_size/2,
                                        query_point(1)-cell_size/2,
                                        query_point(2)-cell_size/2));
    points.push_back(Eigen::Vector3d(query_point(0)-cell_size/2,
                                        query_point(1)+cell_size/2,
                                        query_point(2)+cell_size/2));
    points.push_back(Eigen::Vector3d(query_point(0)-cell_size/2,
                                        query_point(1)-cell_size/2,
                                        query_point(2)+cell_size/2));
    points.push_back(Eigen::Vector3d(query_point(0)+cell_size/2,
                                        query_point(1)+cell_size/2,
                                        query_point(2)-cell_size/2));
    points.push_back(Eigen::Vector3d(query_point(0)-cell_size/2,
                                        query_point(1)+cell_size/2,
                                        query_point(2)-cell_size/2));
    points.push_back(Eigen::Vector3d(query_point(0)+cell_size/2,
                                        query_point(1)-cell_size/2,
                                        query_point(2)+cell_size/2));
    return points;
}

bool planeIntersects(Eigen::Vector3d n, double d, const Eigen::Vector3d& query_point,const std::vector<Eigen::Vector3d>& cube){
    bool allPositive = true;
    bool allNegative = true;
    for (int i=0;i<cube.size();i++){
        double signo = n.dot(cube[i])+d;
        allPositive = allPositive&&(signo>0);
        allNegative = allNegative&&(signo<0);
    }    
    return !allPositive&&!allNegative;
}
bool pointInTriangle(const Eigen::Vector3d& query_point,
                     const Eigen::Vector3d& triangle_vertex_0,
                     const Eigen::Vector3d& triangle_vertex_1,
                     const Eigen::Vector3d& triangle_vertex_2, bool parallel)
{
    // u=P2−P1
    Eigen::Vector3d u = triangle_vertex_1 - triangle_vertex_0;
    // v=P3−P1
    Eigen::Vector3d v = triangle_vertex_2 - triangle_vertex_0;
    // n=u×v
    Eigen::Vector3d n = u.cross(v);
    bool anyProyectionInTriangle=false;
    std::vector<Eigen::Vector3d> cube= cubePoints(query_point);
    for(Eigen::Vector3d vec : cube){
        // w=P−P1
        Eigen::Vector3d w = vec - triangle_vertex_0;
        // Barycentric coordinates of the projection P′of P onto T:
        // γ=[(u×w)⋅n]/n²
        double gamma = u.cross(w).dot(n) / n.dot(n);
        // β=[(w×v)⋅n]/n²
        double beta = w.cross(v).dot(n) / n.dot(n);
        double alpha = 1 - gamma - beta;
        // The point P′ lies inside T if:
        bool proyectionInTriangle= ((0 <= alpha) && (alpha <= 1) &&
                (0 <= beta)  && (beta  <= 1) &&
                (0 <= gamma) && (gamma <= 1));
        anyProyectionInTriangle=anyProyectionInTriangle||proyectionInTriangle;
    }
    
    double d = -n.dot(triangle_vertex_0);

    
    //we consider that the triangle goes through the cell if the proyection of the center 
    //is inside the triangle AND the plane of the triangle intersects the cube of the cell

    return (anyProyectionInTriangle && (parallel||planeIntersects(n,d, query_point,cube)));
}

bool parallel (std::vector<double> &vec){
    return (eq(abs(vec[0]),1)
                &&eq(vec[1],0)
                &&eq(vec[2],0))||
           (eq(vec[0],0)
                &&eq(abs(vec[1]),1)
                &&eq(vec[2],0))||
           (eq(vec[0],0)
                &&eq(vec[1],0)
                &&eq(abs(vec[2]),1));
}
void occupy(std::vector<std::vector<std::vector<int> > >& env,
            std::vector<std::vector<std::vector<double> > > &points, 
            std::vector<std::vector<double> > &normals, int val){

    //Let's occupy the enviroment!
    for(int i= 0;i<points.size();i++){
        //We try to find all the cells that some triangle goes through
        double x1 = points[i][0][0];
        double y1 = points[i][0][1];
        double z1 = points[i][0][2];
        double x2 = points[i][1][0];
        double y2 = points[i][1][1];
        double z2 = points[i][1][2];
        double x3 = points[i][2][0];
        double y3 = points[i][2][1];
        double z3 = points[i][2][2];

        int min_x = (min_val(x1,x2,x3)-env_min_x)/cell_size;
        int min_y = (min_val(y1,y2,y3)-env_min_y)/cell_size;
        int min_z = (min_val(z1,z2,z3)-env_min_z)/cell_size;

        int max_x = (max_val(x1,x2,x3)-env_min_x)/cell_size;
        int max_y = (max_val(y1,y2,y3)-env_min_y)/cell_size;
        int max_z = (max_val(z1,z2,z3)-env_min_z)/cell_size;

        int value=val;
        bool isParallel = parallel(normals[i]);
        if(val==1&&isParallel){
            if(eq(std::fmod(min_val(x1,x2,x3)-env_min_x, cell_size),0)||
                    eq(std::fmod(min_val(y1,y2,y3)-env_min_y, cell_size),0)||
                    eq(std::fmod(min_val(z1,z2,z3)-env_min_z, cell_size),0))
                {
                    value=4;
                }
        }
        for (int row = min_x; row <= max_x && row < env[0].size(); row++)
        {
            for (int col = min_y; col <= max_y && col < env.size(); col++)
            {
                for (int height = min_z; height <= max_z && height < env[0][0].size(); height++)
                {
                    //check if the triangle goes through this cell
                    if (pointInTriangle(Eigen::Vector3d(row * cell_size + env_min_x,
                                                        col * cell_size + env_min_y,
                                                        height * cell_size + env_min_z),
                                        Eigen::Vector3d(x1, y1, z1),
                                        Eigen::Vector3d(x2, y2, z2),
                                        Eigen::Vector3d(x3, y3, z3),isParallel))
                    {
                        env[col][row][height] = value;
                    }
                }
            }
        }
    }
}  
void parse(std::string filename, std::vector<std::vector<std::vector<int> > >& env, int val){
    
    if (FILE *file = fopen(filename.c_str(), "r"))
    {
        //File exists!, keep going!
        fclose(file);
    }else{
        std::cout<< "File " << filename << " does not exist\n";
    }
    
    //first, we count how many triangles there are (we need to do this before reading the data 
    // to create a vector of the right size)
    std::ifstream countfile(filename.c_str());
    std::string line;
    int count = 0;

    while (std::getline(countfile, line)){
        if(line.find("facet normal") != std::string::npos){
            count++;
        }
    }
    //each points[i] contains one the three vertices of triangle i
    std::vector<std::vector<std::vector<double> > >points(count, std::vector<std::vector<double> > (3, std::vector<double>(3)));
    std::vector<std::vector<double> > normals(count, std::vector<double>(3));
    //let's read the data
    std::ifstream infile(filename.c_str());
    std::getline(infile, line);
    int i =0;
    while (line.find("endsolid")==std::string::npos)
        {
            while (line.find("facet normal") == std::string::npos){std::getline(infile, line);}
            size_t pos = line.find("facet");
            line.erase(0, pos + 12);
            std::stringstream ss(line);
            ss >> std::skipws >> normals[i][0];
            ss >> std::skipws >> normals[i][1];
            ss >> std::skipws >> normals[i][2];
            std::getline(infile, line);

            for(int j=0;j<3;j++){
                std::getline(infile, line);
                size_t pos = line.find("vertex ");
                line.erase(0, pos + 7);
                std::stringstream ss(line);
                ss >> std::skipws >> points[i][j][0];
                ss >> std::skipws >> points[i][j][1];
                ss >> std::skipws >> points[i][j][2];
            }
            i++;
            //skipping lines here makes checking for the end of the file more convenient
            std::getline(infile, line);
            std::getline(infile, line);
            while(std::getline(infile, line)&&line.length()==0);
    }
    //OK, we have read the data, let's do something with it
    occupy(env, points, normals, val);

}
void findDimensions(std::string filename){
    if (FILE *file = fopen(filename.c_str(), "r"))
    {
        //File exists!, keep going!
        fclose(file);
    }else{
        std::cout<< "File " << filename << " does not exist\n";
    }

    //let's read the data
    std::string line;
    std::ifstream infile(filename.c_str());
    std::getline(infile, line);
    int i =0;
    while (line.find("endsolid")==std::string::npos)
        {
            while (std::getline(infile, line) && line.find("outer loop") == std::string::npos);

            for(int j=0;j<3;j++){
                double x, y, z;
                std::getline(infile, line);
                size_t pos = line.find("vertex ");
                line.erase(0, pos + 7);
                std::stringstream ss(line);
                ss >> std::skipws >> x;
                ss >> std::skipws >> y;
                ss >> std::skipws >> z;
                env_max_x = env_max_x>=x?env_max_x:x;
                env_max_y = env_max_y>=y?env_max_y:y;
                env_max_z = env_max_z>=z?env_max_z:z;
                env_min_x = env_min_x<=x?env_min_x:x;
                env_min_y = env_min_y<=y?env_min_y:y;
                env_min_z = env_min_z<=z?env_min_z:z;
            }
            i++;
            //skipping three lines here makes checking for the end of the file more convenient
            std::getline(infile, line);
            std::getline(infile, line);
            while(std::getline(infile, line)&&line.length()==0);
    }
}
void openFoam_to_gaden(std::string filename, std::vector<std::vector<std::vector<int> > >& env)
{

	//let's parse the file
	std::ifstream infile(filename.c_str());
	std::string line;

	//ignore the first line (column names)
	std::getline(infile, line);
    std::vector<std::vector<std::vector<double> > > U(env[0].size(), std::vector<std::vector<double> >(env.size(), std::vector<double>(env[0][0].size())));
    std::vector<std::vector<std::vector<double> > > V(env[0].size(), std::vector<std::vector<double> >(env.size(), std::vector<double>(env[0][0].size())));
    std::vector<std::vector<std::vector<double> > > W(env[0].size(), std::vector<std::vector<double> >(env.size(), std::vector<double>(env[0][0].size())));
    std::vector<double> v(6);
	int x_idx = 0;
	int y_idx = 0;
	int z_idx = 0;
	while (std::getline(infile, line))
	{
		if (line.length()!=0)
		{
			for (int i = 0; i < 6; i++)
			{
				size_t pos = line.find(",");
				v[i] = atof(line.substr(0, pos).c_str());
				line.erase(0, pos + 1);
			}
			//assign each of the points we have information about to the nearest cell
			x_idx = (v[3] - env_min_x) / cell_size;
			y_idx = (v[4] - env_min_y) / cell_size;
			z_idx = (v[5] - env_min_z) / cell_size;
			U[x_idx][y_idx][z_idx] = v[0];
			V[x_idx][y_idx][z_idx] = v[1];
			W[x_idx][y_idx][z_idx] = v[2];
		}
	}
    printWind(U,V,W,filename);
}

void fill(int x, int y, int z, std::vector<std::vector<std::vector<int> > >& env){
    std::cout<<"Filling...\n";
    std::queue<Eigen::Vector3i> q;
    q.push(Eigen::Vector3i(x, y, z));
    env[x][y][1]=3;
    while(!q.empty()){
        Eigen::Vector3i point = q.front();
        q.pop();
        if(point[0]+1<env.size()&&env[point[0]+1][point[1]][point[2]]==0){ // x+1, y, z
            env[point[0]+1][point[1]][point[2]]=3;
            q.push(Eigen::Vector3i(point[0]+1,point[1],point[2]));
        }
        if(point[0]>0&&env[point[0]-1][point[1]][point[2]]==0){ //x-1, y, z
            env[point[0]-1][point[1]][point[2]]=3;
            q.push(Eigen::Vector3i(point[0]-1,point[1],point[2]));
        }
        if(point[1]+1<env[0].size()&&env[point[0]][point[1]+1][point[2]]==0){ //x, y+1, z
            env[point[0]][point[1]+1][point[2]]=3;
            q.push(Eigen::Vector3i(point[0],point[1]+1,point[2]));
        }
        if(point[1]>0&&env[point[0]][point[1]-1][point[2]]==0){ //x, y-1, z
            env[point[0]][point[1]-1][point[2]]=3;
            q.push(Eigen::Vector3i(point[0],point[1]-1,point[2]));
        }
        if(point[2]+1<env[0][0].size()&&env[point[0]][point[1]][point[2]+1]==0){ //x, y, z+1
            env[point[0]][point[1]][point[2]+1]=3;
            q.push(Eigen::Vector3i(point[0],point[1],point[2]+1));
        }
        if(point[2]>0&&env[point[0]][point[1]][point[2]-1]==0){ //x, y, z-1
            env[point[0]][point[1]][point[2]-1]=3;
            q.push(Eigen::Vector3i(point[0],point[1],point[2]-1));
        }
    }
}

void clean(std::vector<std::vector<std::vector<int> > >& env){
    std::stack<Eigen::Vector3i> st;
    for(int col=0;col<env.size();col++){
        for(int row=0;row<env[0].size();row++){
            for(int height=0;height<env[0][0].size();height++){
                if(env[col][row][height]==4){
                    if((col<env.size()-1&&env[col+1][row][height]==3)||
                            (row<env[0].size()-1&&env[col][row+1][height]==3)||
                            (height<env[0][0].size()-1&&env[col][row][height+1]==3))
                    {
                        env[col][row][height]=3;
                    }else
                    {
                        env[col][row][height]=1;
                        if((col<env.size()-1&&env[col+1][row][height]==4)&&
                                (row<env[0].size()-1&&env[col][row+1][height]==4)&&
                                (height<env[0][0].size()-1&&env[col][row][height+1]==4))
                        {
                            st.push(Eigen::Vector3i(col, row, height));
                        }
                    }
                    
                }
                
            }
        }
    }
    while(!st.empty()){
        Eigen::Vector3i point=st.top();
        st.pop();
        int col=point[0];int row=point[1];int height=point[2];
        if(col<env.size()-1&&(env[col+1][row][height]==3)&&
                (row<env[0].size()-1&&env[col][row+1][height]==3))
        {
            env[col][row][height]=3;
        }else
        {
            env[col][row][height]=1;
        }
    }
}
int main(int argc, char **argv){
    ros::init(argc, argv, "preprocessing");
    int numModels;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("preprocessing_done",5,true);

    private_nh.param<double>("cell_size", cell_size, 0.1); //size of the cells
    private_nh.param<int>("number_of_models", numModels, 1); // number of CAD models
    
    std::vector<std::string> CADfiles;     
    for(int i = 0; i< numModels; i++){
        std::string paramName = boost::str( boost::format("model_%i") % i); //each of the stl models
        std::string filename;
        private_nh.param<std::string>(paramName, filename, "");
        CADfiles.push_back(filename.c_str());
    }

    //stl file with the model of the outlets
    std::string outlet;
    private_nh.param<std::string>("outlets_model", outlet, "");

    //path to the point cloud files with the wind data
    std::string windFileName;
    private_nh.param<std::string>("wind_files", windFileName, "");

    //path to the csv file where we want to write the occupancy map
    std::string output;
    private_nh.param<std::string>("output_path", output, "");

    //OCCUPANCY

    for (int i = 0; i < CADfiles.size(); i++)
    {
        findDimensions(CADfiles[i]);
    }

    std::vector<std::vector<std::vector<int> > > env((env_max_y - env_min_y) / cell_size,
                                                    std::vector<std::vector<int> >((env_max_x - env_min_x) / cell_size,
                                                                                    std::vector<int>((env_max_z - env_min_z) / cell_size, 0)));
    for (int i = 0; i < numModels; i++)
    {
        parse(CADfiles[i], env, 1);
    }
    parse(outlet, env, 2);

    double empty_point_x;
    private_nh.param<double>("empty_point_x", empty_point_x, 1);
    double empty_point_y;
    private_nh.param<double>("empty_point_y", empty_point_y, 1);
    double empty_point_z;
    private_nh.param<double>("empty_point_z", empty_point_z, 1);
    fill((empty_point_x-env_min_x)/cell_size,
        (empty_point_y-env_min_y)/cell_size,
        (empty_point_z-env_min_z)/cell_size, 
        env);
    clean(env);
    //output - path, occupancy vector, scale
    printEnv(boost::str(boost::format("%s/OccupancyGrid3D.csv") % output.c_str()), env, 1);
    printEnv(boost::str(boost::format("%s/occupancy.pgm") % output.c_str()), env, 10);

    //WIND
    int idx = 0;
    while (FILE *file = fopen(boost::str(boost::format("%s_%i.csv") % windFileName % idx).c_str(), "r"))
    {
        fclose(file);
        openFoam_to_gaden(boost::str(boost::format("%s_%i.csv") % windFileName % idx).c_str(), env);
        idx++;
    }
    

    ROS_INFO("Preprocessing done");
    
}
