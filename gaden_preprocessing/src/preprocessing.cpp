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
                                            (env[row][col][height]==3?2:
                                            (env[row][col][height]==5?0:
                                                env[row][col][height])))
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

void printYaml(std::string output){
    std::ofstream yaml(boost::str(boost::format("%s/occupancy.yaml") % output.c_str()));
    yaml << "image: occupancy.pgm\n" 
        << "resolution: " << cell_size/10 
        << "\norigin: [" << env_min_x << ", " << env_min_y << ", " << env_min_z << "]\n"
        << "occupied_thresh: 0.9\n" 
        << "free_thresh: 0.1\n" 
        << "negate: 0";
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
    return std::abs(x-y)<0.0001;
}

std::vector<Eigen::Vector3d> cubePoints(const Eigen::Vector3d &query_point){
    std::vector<Eigen::Vector3d> points;
    points.push_back(query_point);
    points.push_back(Eigen::Vector3d(query_point.x()-cell_size/2,
                                            query_point.y()-cell_size/2,
                                            query_point.z()-cell_size/2));
    points.push_back(Eigen::Vector3d(query_point.x()-cell_size/2,
                                            query_point.y()-cell_size/2,
                                            query_point.z()+cell_size/2));
    points.push_back(Eigen::Vector3d(query_point.x()-cell_size/2,
                                            query_point.y()+cell_size/2,
                                            query_point.z()-cell_size/2));
    points.push_back(Eigen::Vector3d(query_point.x()-cell_size/2,
                                            query_point.y()+cell_size/2,
                                            query_point.z()+cell_size/2));
    points.push_back(Eigen::Vector3d(query_point.x()+cell_size/2,
                                            query_point.y()-cell_size/2,
                                            query_point.z()-cell_size/2));
    points.push_back(Eigen::Vector3d(query_point.x()+cell_size/2,
                                            query_point.y()-cell_size/2,
                                            query_point.z()+cell_size/2));
    points.push_back(Eigen::Vector3d(query_point.x()+cell_size/2,
                                            query_point.y()+cell_size/2,
                                            query_point.z()-cell_size/2));
    points.push_back(Eigen::Vector3d(query_point.x()+cell_size/2,
                                            query_point.y()+cell_size/2,
                                            query_point.z()+cell_size/2));
    return points;
}

bool planeIntersects(Eigen::Vector3d& n, const Eigen::Vector3d& planePoint,const std::vector<Eigen::Vector3d>& cube){
    bool allPositive = true;
    bool allNegative = true;
    for (int i=0;i<cube.size();i++){
        double signo = n.dot(cube[i]-planePoint);
        allPositive = allPositive&&(signo>0);
        allNegative = allNegative&&(signo<0);
    } 
    return !allPositive&&!allNegative;
}

bool pointInTriangle(const Eigen::Vector3d& query_point,
                     const Eigen::Vector3d& triangle_vertex_0,
                     const Eigen::Vector3d& triangle_vertex_1,
                     const Eigen::Vector3d& triangle_vertex_2,
                     bool parallel)
{
    // u=P2−P1
    Eigen::Vector3d u = triangle_vertex_1 - triangle_vertex_0;
    // v=P3−P1
    Eigen::Vector3d v = triangle_vertex_2 - triangle_vertex_0;
    // n=u×v
    Eigen::Vector3d n = u.cross(v);
    bool anyProyectionInTriangle=false;
    std::vector<Eigen::Vector3d> cube= cubePoints(query_point);
    for(const Eigen::Vector3d &vec : cube){
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

    n.normalize();
    
    //we consider that the triangle goes through the cell if the proyection of the center 
    //is inside the triangle AND the plane of the triangle intersects the cube of the cell
    
    return (anyProyectionInTriangle && (parallel||planeIntersects(n, triangle_vertex_0, cube)));
}

bool parallel (std::vector<double> &vec){
    return (eq(vec[1],0)
                &&eq(vec[2],0))||
           (eq(vec[0],0)
                &&eq(vec[2],0))||
           (eq(vec[0],0)
                &&eq(vec[1],0));
}

void occupy(std::vector<std::vector<std::vector<int> > >& env,
            std::vector<std::vector<std::vector<double> > > &points, 
            std::vector<std::vector<double> > &normals, int val){

    //Let's occupy the enviroment!
    for(int i= 0;i<points.size();i++){
        //We try to find all the cells that some triangle goes through
        int x1 = roundf((points[i][0][0]-env_min_x)*(roundFactor))/(cell_size*(roundFactor));
        int y1 = roundf((points[i][0][1]-env_min_y)*(roundFactor))/(cell_size*(roundFactor));
        int z1 = roundf((points[i][0][2]-env_min_z)*(roundFactor))/(cell_size*(roundFactor));
        int x2 = roundf((points[i][1][0]-env_min_x)*(roundFactor))/(cell_size*(roundFactor));
        int y2 = roundf((points[i][1][1]-env_min_y)*(roundFactor))/(cell_size*(roundFactor));
        int z2 = roundf((points[i][1][2]-env_min_z)*(roundFactor))/(cell_size*(roundFactor));
        int x3 = roundf((points[i][2][0]-env_min_x)*(roundFactor))/(cell_size*(roundFactor));
        int y3 = roundf((points[i][2][1]-env_min_y)*(roundFactor))/(cell_size*(roundFactor));
        int z3 = roundf((points[i][2][2]-env_min_z)*(roundFactor))/(cell_size*(roundFactor));

        int min_x = min_val(x1,x2,x3);
        int min_y = min_val(y1,y2,y3);
        int min_z = min_val(z1,z2,z3);

        int max_x = max_val(x1,x2,x3);
        int max_y = max_val(y1,y2,y3);
        int max_z = max_val(z1,z2,z3);

        bool isParallel =parallel(normals[i]);
        bool xLimit = eq(std::fmod(max_val(points[i][0][0],points[i][1][0],points[i][2][0])-env_min_x, cell_size),0)||eq(std::fmod(max_val(points[i][0][0],points[i][1][0],points[i][2][0])-env_min_x, cell_size),cell_size);
        bool yLimit = eq(std::fmod(max_val(points[i][0][1],points[i][1][1],points[i][2][1])-env_min_y, cell_size),0)||eq(std::fmod(max_val(points[i][0][1],points[i][1][1],points[i][2][1])-env_min_y, cell_size),cell_size);
        bool zLimit = eq(std::fmod(max_val(points[i][0][2],points[i][1][2],points[i][2][2])-env_min_z, cell_size),0)||eq(std::fmod(max_val(points[i][0][2],points[i][1][2],points[i][2][2])-env_min_z, cell_size),cell_size);

        if(x1<env[0].size()&&y1<env.size()&&z1<env[0][0].size()){
            if((xLimit&&x1==max_x)||
                (yLimit&&y1==max_y)||
                (zLimit&&z1==max_z)
                &&!env[y1][x1][z1]==1){
                    env[y1][x1][z1]=(val==1?4:val);
            }else{
                env[y1][x1][z1] = val;
            }
        }

        if(x2<env[0].size()&&y2<env.size()&&z2<env[0][0].size()){
            if((xLimit&&x2==max_x)||
                (yLimit&&y2==max_y)||
                (zLimit&&z2==max_z)
                &&!env[y2][x2][z2]==1){
                    env[y2][x2][z2]=(val==1?4:val);
            }else{
                env[y2][x2][z2] = val;
            }
        }

        if(x3<env[0].size()&&y3<env.size()&&z3<env[0][0].size()){
            if((xLimit&&x3==max_x)||
                (yLimit&&y3==max_y)||
                (zLimit&&z3==max_z)
                &&!env[y3][x3][z3]==1){
                    env[y3][x3][z3]=(val==1?4:val);
            }else{
                env[y3][x3][z3] = val;
            }
        }
        
        for (int row = min_x; row <= max_x && row < env[0].size(); row++)
        {
            for (int col = min_y; col <= max_y && col < env.size(); col++)
            {
                for (int height = min_z; height <= max_z && height < env[0][0].size(); height++)
                {
                    //check if the triangle goes through this cell
                    if (pointInTriangle(Eigen::Vector3d(row * cell_size + env_min_x+cell_size/2,
                                                        col * cell_size + env_min_y+cell_size/2,
                                                        height * cell_size + env_min_z+cell_size/2),
                                        Eigen::Vector3d(points[i][0][0], points[i][0][1], points[i][0][2]),
                                        Eigen::Vector3d(points[i][1][0], points[i][1][1], points[i][1][2]),
                                        Eigen::Vector3d(points[i][2][0], points[i][2][1], points[i][2][2]),
                                        isParallel))
                    {
                        if((xLimit&&row==max_x)||
                            (yLimit&&col==max_y)||
                            (zLimit&&height==max_z)
                            &&!env[col][row][height]==1){
                                env[col][row][height]=(val==1?4:val);
                        }else{
                            env[col][row][height] = val;
                        }
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
            double aux;
            std::stringstream ss(line);
            ss >> std::skipws >>  aux; 
            normals[i][0] = roundf(aux * roundFactor) / roundFactor;
            ss >> std::skipws >>  aux; 
            normals[i][1] = roundf(aux * roundFactor) / roundFactor;
            ss >> std::skipws >>  aux; 
            normals[i][2] = roundf(aux * roundFactor) / roundFactor;
            std::getline(infile, line);

            for(int j=0;j<3;j++){
                std::getline(infile, line);
                size_t pos = line.find("vertex ");
                line.erase(0, pos + 7);
                std::stringstream ss(line);
                ss >> std::skipws >>  aux; 
                points[i][j][0] = roundf(aux * roundFactor) / roundFactor;
                ss >> std::skipws >>  aux; 
                points[i][j][1] = roundf(aux * roundFactor) / roundFactor;
                ss >> std::skipws >>  aux; 
                points[i][j][2] = roundf(aux * roundFactor) / roundFactor;
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
                double aux;
                ss >> std::skipws >>  aux; 
                x = roundf(aux * roundFactor) / roundFactor;
                ss >> std::skipws >>  aux; 
                y = roundf(aux * roundFactor) / roundFactor;
                ss >> std::skipws >>  aux; 
                z = roundf(aux * roundFactor) / roundFactor;
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
			x_idx = roundf((v[3] - env_min_x) / cell_size*roundFactor)/roundFactor;
			y_idx = roundf((v[4] - env_min_y) / cell_size*roundFactor)/roundFactor;
			z_idx = roundf((v[5] - env_min_z) / cell_size*roundFactor)/roundFactor;
			U[x_idx][y_idx][z_idx] = v[0];
			V[x_idx][y_idx][z_idx] = v[1];
			W[x_idx][y_idx][z_idx] = v[2];
		}
	}
    printWind(U,V,W,filename);
}

void fill(int x, int y, int z, std::vector<std::vector<std::vector<int> > >& env, int val, int empty){
    std::queue<Eigen::Vector3i> q;
    q.push(Eigen::Vector3i(x, y, z));
    env[x][y][z]=val;
    while(!q.empty()){
        Eigen::Vector3i point = q.front();
        q.pop();
        if(point[0]+1<env.size()&&env[point[0]+1][point[1]][point[2]]==empty){ // x+1, y, z
            env[point[0]+1][point[1]][point[2]]=val;
            q.push(Eigen::Vector3i(point[0]+1,point[1],point[2]));
        }
        if(point[0]>0&&env[point[0]-1][point[1]][point[2]]==empty){ //x-1, y, z
            env[point[0]-1][point[1]][point[2]]=val;
            q.push(Eigen::Vector3i(point[0]-1,point[1],point[2]));
        }
        if(point[1]+1<env[0].size()&&env[point[0]][point[1]+1][point[2]]==empty){ //x, y+1, z
            env[point[0]][point[1]+1][point[2]]=val;
            q.push(Eigen::Vector3i(point[0],point[1]+1,point[2]));
        }
        if(point[1]>0&&env[point[0]][point[1]-1][point[2]]==empty){ //x, y-1, z
            env[point[0]][point[1]-1][point[2]]=val;
            q.push(Eigen::Vector3i(point[0],point[1]-1,point[2]));
        }
        if(point[2]+1<env[0][0].size()&&env[point[0]][point[1]][point[2]+1]==empty){ //x, y, z+1
            env[point[0]][point[1]][point[2]+1]=val;
            q.push(Eigen::Vector3i(point[0],point[1],point[2]+1));
        }
        if(point[2]>0&&env[point[0]][point[1]][point[2]-1]==empty){ //x, y, z-1
            env[point[0]][point[1]][point[2]-1]=val;
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
                            (height<env[0][0].size()-1&&env[col][row][height+1]==3)||
                            (col<env.size()-1&&row<env[0].size()-1&&env[col+1][row+1][height]==3
                                &&env[col][row+1][height]==4
                                &&env[col+1][row][height]==4))
                    {
                        env[col][row][height]=3;
                    }else
                    {
                        env[col][row][height]=1;
                    }
                    
                }
                
            }
        }
    }
}
int main(int argc, char **argv){
    ros::init(argc, argv, "preprocessing");
    int numModels;
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("preprocessing_done",5,true);

    private_nh.param<double>("cell_size", cell_size, 1); //size of the cells

    roundFactor=1000.0/cell_size;
    //stl file with the model of the outlets
    std::string outlet; int numOutletModels;

    //path to the csv file where we want to write the occupancy map
    std::string output;
    private_nh.param<std::string>("output_path", output, "");

    //--------------------------

        //OCCUPANCY

    //--------------------------

    private_nh.param<int>("number_of_models", numModels, 2); // number of CAD models
    
    std::vector<std::string> CADfiles;     
    for(int i = 0; i< numModels; i++){
        std::string paramName = boost::str( boost::format("model_%i") % i); //each of the stl models
        std::string filename;
        private_nh.param<std::string>(paramName, filename, "");
        CADfiles.push_back(filename.c_str());
    }

    for (int i = 0; i < CADfiles.size(); i++)
    {
        findDimensions(CADfiles[i]);
    }

    //x and y are interchanged!!!!!! it goes env[y][x][z]
    //I cannot for the life of me remember why I did that, but there must have been a reason
    std::vector<std::vector<std::vector<int> > > env(ceil((env_max_y-env_min_y)*(roundFactor)/(cell_size*(roundFactor))),
                                                    std::vector<std::vector<int> >(ceil((env_max_x - env_min_x)*(roundFactor)/(cell_size*(roundFactor))),
                                                                                    std::vector<int>(ceil((env_max_z - env_min_z)*(roundFactor)/(cell_size*(roundFactor))), 0)));

    for (int i = 0; i < numModels; i++)
    {
        parse(CADfiles[i], env, 1);
    }
      

    double empty_point_x;
    private_nh.param<double>("empty_point_x", empty_point_x, 1);
    double empty_point_y;
    private_nh.param<double>("empty_point_y", empty_point_y, 1);
    double empty_point_z;
    private_nh.param<double>("empty_point_z", empty_point_z, 1);

    std::cout<<"Filling...\n";
    fill((empty_point_y-env_min_y)/cell_size,
        (empty_point_x-env_min_x)/cell_size,
        (empty_point_z-env_min_z)/cell_size, 
        env, 3, 0);
    clean(env);


    printEnv(boost::str(boost::format("%s/occupancy.pgm") % output.c_str()), env, 10);
    
    //--------------------------

        //OUTLETS

    //--------------------------

    private_nh.param<int>("number_of_outlet_models", numOutletModels, 1); // number of CAD models

    std::vector<std::string> outletFiles;     
    for(int i = 0; i< numOutletModels; i++){
        std::string paramName = boost::str( boost::format("outlets_model_%i") % i); //each of the stl models
        std::string filename;
        private_nh.param<std::string>(paramName, filename, "");
        outletFiles.push_back(filename.c_str());
    }

    for (int i=0;i<numOutletModels; i++){
        parse(outletFiles[i], env, 2);
    }  

    fill((empty_point_y-env_min_y)/cell_size,
        (empty_point_x-env_min_x)/cell_size,
        (empty_point_z-env_min_z)/cell_size, 
        env, 5, 3);

    //output - path, occupancy vector, scale
    printEnv(boost::str(boost::format("%s/OccupancyGrid3D.csv") % output.c_str()), env, 1);
    printYaml(output);

    //-------------------------

        //WIND

    //-------------------------

    bool uniformWind;
    private_nh.param<bool>("uniformWind", uniformWind, false);

    //path to the point cloud files with the wind data
    std::string windFileName;
    private_nh.param<std::string>("wind_files", windFileName, "");
    int idx = 0;

    if(uniformWind){
        
        //let's parse the file
        std::ifstream infile(windFileName);
        std::string line;

        std::vector<std::vector<std::vector<double> > > U(env[0].size(), std::vector<std::vector<double> >(env.size(), std::vector<double>(env[0][0].size())));
        std::vector<std::vector<std::vector<double> > > V(env[0].size(), std::vector<std::vector<double> >(env.size(), std::vector<double>(env[0][0].size())));
        std::vector<std::vector<std::vector<double> > > W(env[0].size(), std::vector<std::vector<double> >(env.size(), std::vector<double>(env[0][0].size())));
        while(std::getline(infile, line)){
            std::vector<double> v;
            for (int i = 0; i < 3; i++)
			{
				size_t pos = line.find(",");
				v.push_back(atof(line.substr(0, pos).c_str()));
				line.erase(0, pos + 1);
			}

            for(int i = 0; i< env[0].size();i++){
                for(int j = 0; j< env.size();j++){
                    for(int k = 0; k< env[0][0].size();k++){
                        if(env[j][i][k]==5){
                            U[i][j][k] = v[0];
                            V[i][j][k] = v[1];
                            W[i][j][k] = v[2];
                        }
                    }
                }
            }
            printWind(U,V,W, boost::str(boost::format("%s_%i.csv") % windFileName % idx).c_str());
            idx++;
        }
    }else{
        while (FILE *file = fopen(boost::str(boost::format("%s_%i.csv") % windFileName % idx).c_str(), "r"))
        {
            fclose(file);
            openFoam_to_gaden(boost::str(boost::format("%s_%i.csv") % windFileName % idx).c_str(), env);
            idx++;
        }
    }
    
    

    ROS_INFO("Preprocessing done");
    std_msgs::Bool b;
    b.data=true;
    ros::Rate r(0.1);
    while(ros::ok()){
        pub.publish(b);
        r.sleep();
    }
    
}
