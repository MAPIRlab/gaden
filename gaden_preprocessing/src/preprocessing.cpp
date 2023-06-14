#include <gaden_preprocessing/Gaden_preprocessing.h>
#include <gaden_preprocessing/TriangleBoxIntersection.h>

#include <string>
#include <fstream>
#include <stdlib.h> 
#include <sstream>
#include <iostream>
#include <boost/format.hpp>
#include <queue>
#include <stack>
#include <stdint.h>

#ifdef GENERATE_COPPELIA_SCENE
#define SIM_REMOTEAPICLIENT_OBJECTS
#include <RemoteAPIClient.h>
#endif

int main(int argc, char **argv){
    rclcpp::init(argc, argv);


    std::shared_ptr<Gaden_preprocessing> node = std::make_shared<Gaden_preprocessing>();
    node->parseMainModels();
    if(!rclcpp::ok())
        return -1;
    node->parseOutletModels();
    
    //Mark all the empty cells reachable from the empty_point as aux_empty
    //the ones that cannot be reached will be marked as occupied when printing
    node->fill();
    
    //get rid of the cells marked as "edge", since those are not truly occupied
    node->clean();

    node->processWind(); 
    node-> generateOutput();
    

    RCLCPP_INFO(node->get_logger(), "Preprocessing done");
    std_msgs::msg::Bool b;
    b.data=true;
    node->jobDone_pub->publish(b);
    return 0;
}



void Gaden_preprocessing::parseMainModels()
{
    int numModels = declare_parameter<int>("number_of_models", 0);

    bool generateCoppeliaScene = declare_parameter<bool>("generateCoppeliaScene", false);
#ifdef GENERATE_COPPELIA_SCENE
    RemoteAPIClient client;
    if(generateCoppeliaScene)
        client.getObject().sim().stopSimulation();
#else
    if(generateCoppeliaScene)
    {
        RCLCPP_ERROR(get_logger(), "You are trying to generate a coppelia scene, but compiled gaden_preprocessing without coppelia support. Either disable the request in the launch file or compile with coppelia support. \nYou can enable the generation in the CMakeLists.txt file of the preprocessing package.");
        rclcpp::shutdown();
        return;
    }
#endif


    std::vector<std::string> CADfiles;     
    for(int i = 0; i< numModels; i++){
        std::string paramName = boost::str( boost::format("model_%i") % i); //each of the stl models
        std::string filename;
        filename = declare_parameter<std::string>(paramName, "");
        CADfiles.push_back(filename.c_str());
    }

    for (int i = 0; i < CADfiles.size(); i++)
    {
        findDimensions(CADfiles[i]);
    }

    //x and y are interchanged!!!!!! it goes env[y][x][z]
    //I cannot for the life of me remember why I did that, but there must have been a reason
    env = std::vector<std::vector<std::vector<int> > > (ceil((env_max_y-env_min_y)*(roundFactor)/(cell_size*(roundFactor))),
                                                    std::vector<std::vector<int> >(ceil((env_max_x - env_min_x)*(roundFactor)/(cell_size*(roundFactor))),
                                                                                    std::vector<int>(ceil((env_max_z - env_min_z)*(roundFactor)/(cell_size*(roundFactor))), 0)));

    for (int i = 0; i < numModels; i++)
    {
        parse(CADfiles[i], cell_state::occupied);
#ifdef GENERATE_COPPELIA_SCENE
    if(generateCoppeliaScene)
    {
        int result = -1;
        do{
            result = client.getObject().sim().importShape(0, CADfiles[i], 0, 0.0001f, 1);
        }while (result==-1);
    }
#endif
    }

    std::string outputFolder = declare_parameter<std::string>("output_path", "");

#ifdef GENERATE_COPPELIA_SCENE
    if(generateCoppeliaScene)
        client.getObject().sim().saveScene(boost::str(boost::format("%s/coppeliaScene.ttt") % outputFolder));
#endif
}


void Gaden_preprocessing::parseOutletModels()
{

    int numOutletModels = declare_parameter<int>("number_of_outlet_models", 1); // number of CAD models

    std::vector<std::string> outletFiles;     
    for(int i = 0; i< numOutletModels; i++){
        std::string paramName = boost::str( boost::format("outlets_model_%i") % i); //each of the stl models
        std::string filename;
        filename = declare_parameter<std::string>(paramName, "");
        outletFiles.push_back(filename.c_str());
    }

    for (int i=0;i<numOutletModels; i++){
        parse(outletFiles[i], cell_state::outlet);
    }  
}

bool Gaden_preprocessing::compare_cell(int x, int y, int z, cell_state value){
    if(x<0 || x>=env.size() ||
        y<0 || y>=env[0].size() ||
        z<0 || z>=env[0][0].size()){
            return false;
    }
    else{
        return env[x][y][z]==value;
    }
}

void Gaden_preprocessing::changeWorldFile(const std::string& filename){
    std::ifstream input(filename);
    std::stringstream ss;
    std::string line;

    float floor_height = get_parameter("floor_height").as_double();
    while(getline(input, line)){
        if(line.substr(0,8)=="floorMap"){
            //ignore the floorMap bit, we are replacing it entirely
            while(getline(input, line) && line!=")"){}

            ss<< 
            "floorMap                     # load an environment bitmap\n"<<
            "(\n"<<
                "  name \"SimulatedMap\"\n"<< 
                "  bitmap \"../../occupancy.pgm\"\n"<<
                "  size ["<<(env_max_x-env_min_x)<<" "<<(env_max_y-env_min_y)<<" "<<(env_max_z-env_min_z) <<"]           #m \n"<< 
                "  pose ["<<(env_max_x-env_min_x)/2+env_min_x<<" "<<(env_max_y-env_min_y)/2+env_min_y<<" "<<floor_height<<" 0]    #Coordinates (m) of the Center of the image_map\n"<<
            ")\n";
        }
        else{
            ss<<line<<"\n";
        }
    }
    input.close();
    std::ofstream out(filename);
    out<<ss.rdbuf();
    out.close();
}

void Gaden_preprocessing::printMap(std::string filename, int scale, bool block_outlets){
    std::ofstream outfile(filename.c_str());
    outfile << "P2\n"
            << scale *  env[0].size() << " " << scale * env.size() << "\n" <<"1\n";
    //things are repeated to scale them up (the image is too small!)

    float floor_height = declare_parameter<float>("floor_height", 0);
    int height = (floor_height-env_min_z)/cell_size; //a xy slice of the 3D environment is used as a geometric map for navigation
    
    for (int row = env.size()-1; row >= 0; row--)
    {
        for (int j = 0; j < scale; j++)
        {
            for (int col = 0; col <env[0].size() ; col++)
            {
                for (int i = 0; i < scale; i++)
                {
                    auto& cell = env[row][col][height];
                    bool outletTerm = cell == cell_state::outlet && !block_outlets;
                    outfile << ( cell == cell_state::empty || outletTerm ? 1 : 0) << " ";
                }
            }
            outfile << "\n";
        }
    }
    outfile.close();

}

void Gaden_preprocessing::printEnv(std::string filename, int scale)
{
    std::ofstream outfile(filename.c_str());
    
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
                        outfile << (env[row][col][height]==cell_state::empty? 0 :
                                (env[row][col][height]==cell_state::outlet? 2 :
                                1))
                                << " ";
                    }
                }
                outfile << "\n";
            }
        }
        outfile << ";\n";
    }
    outfile.close();
}

void Gaden_preprocessing::printWind(const std::vector<double>& U,
                const std::vector<double>& V,
                const std::vector<double>& W, std::string filename){
    
    std::ofstream fileU(boost::str(boost::format("%s_U") % filename).c_str());
    std::ofstream fileV(boost::str(boost::format("%s_V") % filename).c_str());
    std::ofstream fileW(boost::str(boost::format("%s_W") % filename).c_str());
    
    //this code is a header to let the filament_simulator know the file is in binary
    int code=999;

    fileU.write((char*) &code, sizeof(int));
    fileV.write((char*) &code, sizeof(int));
    fileW.write((char*) &code, sizeof(int));

    fileU.write((char*) U.data(), sizeof(double) * U.size());
    fileV.write((char*) V.data(), sizeof(double) * V.size());
    fileW.write((char*) W.data(), sizeof(double) * W.size());

    fileU.close();
    fileV.close();
    fileW.close();
}

void Gaden_preprocessing::printYaml(std::string output){
    std::ofstream yaml(boost::str(boost::format("%s/occupancy.yaml") % output.c_str()));
    yaml << "image: occupancy.pgm\n" 
        << "resolution: " << cell_size/10 
        << "\norigin: [" << env_min_x << ", " << env_min_y << ", " << 0 << "]\n"
        << "occupied_thresh: 0.9\n" 
        << "free_thresh: 0.1\n" 
        << "negate: 0";
    yaml.close();
}



std::array<tf2::Vector3, 9> Gaden_preprocessing::cubePoints(const tf2::Vector3 &query_point){
    std::array<tf2::Vector3, 9> points;
    points[0] = (query_point);
    points[1] = (tf2::Vector3(query_point.x()-cell_size/2,
                                            query_point.y()-cell_size/2,
                                            query_point.z()-cell_size/2));
    points[2] = (tf2::Vector3(query_point.x()-cell_size/2,
                                            query_point.y()-cell_size/2,
                                            query_point.z()+cell_size/2));
    points[3] = (tf2::Vector3(query_point.x()-cell_size/2,
                                            query_point.y()+cell_size/2,
                                            query_point.z()-cell_size/2));
    points[4] = (tf2::Vector3(query_point.x()-cell_size/2,
                                            query_point.y()+cell_size/2,
                                            query_point.z()+cell_size/2));
    points[5] = (tf2::Vector3(query_point.x()+cell_size/2,
                                            query_point.y()-cell_size/2,
                                            query_point.z()-cell_size/2));
    points[6] = (tf2::Vector3(query_point.x()+cell_size/2,
                                            query_point.y()-cell_size/2,
                                            query_point.z()+cell_size/2));
    points[7] = (tf2::Vector3(query_point.x()+cell_size/2,
                                            query_point.y()+cell_size/2,
                                            query_point.z()-cell_size/2));
    points[8] = (tf2::Vector3(query_point.x()+cell_size/2,
                                            query_point.y()+cell_size/2,
                                            query_point.z()+cell_size/2));
    return points;
}

bool Gaden_preprocessing::pointInTriangle(const tf2::Vector3& query_point,
                     const tf2::Vector3& triangle_vertex_0,
                     const tf2::Vector3& triangle_vertex_1,
                     const tf2::Vector3& triangle_vertex_2)
{
    // u=P2−P1
    tf2::Vector3 u = triangle_vertex_1 - triangle_vertex_0;
    // v=P3−P1
    tf2::Vector3 v = triangle_vertex_2 - triangle_vertex_0;
    // n=u×v
    tf2::Vector3 n = u.cross(v);
    bool anyProyectionInTriangle=false;
    std::array<tf2::Vector3, 9> cube= cubePoints(query_point);
    for(const tf2::Vector3 &vec : cube){
        // w=P−P1
        tf2::Vector3 w = vec - triangle_vertex_0;
        // Barycentric coordinates of the projection P′of P onto T:
        // γ=[(u×w)⋅n]/n²
        float gamma = u.cross(w).dot(n) / n.dot(n);
        // β=[(w×v)⋅n]/n²
        float beta = w.cross(v).dot(n) / n.dot(n);
        float alpha = 1 - gamma - beta;
        // The point P′ lies inside T if:
        bool proyectionInTriangle= ((0 <= alpha) && (alpha <= 1) &&
                (0 <= beta)  && (beta  <= 1) &&
                (0 <= gamma) && (gamma <= 1));
        anyProyectionInTriangle=anyProyectionInTriangle||proyectionInTriangle;
    }

    n.normalize();
    
    //we consider that the triangle goes through the cell if the proyection of the center 
    //is inside the triangle AND the plane of the triangle intersects the cube of the cell
    
    return anyProyectionInTriangle;
}



void Gaden_preprocessing::occupy(std::vector<Triangle> &triangles,
            const std::vector<tf2::Vector3> &normals,
            cell_state value_to_write){

    std::cout<<"Processing the mesh...\n0%\n";
    int numberOfProcessedTriangles=0; //for logging, doesn't actually do anything
    std::mutex mtx;
    //Let's occupy the enviroment!
    #pragma omp parallel for
    for(int i= 0;i<triangles.size();i++){
        //We try to find all the cells that some triangle goes through
        int x1 = roundf((triangles[i].p1.x()-env_min_x)*(roundFactor))/(cell_size*(roundFactor));
        int y1 = roundf((triangles[i].p1.y()-env_min_y)*(roundFactor))/(cell_size*(roundFactor));
        int z1 = roundf((triangles[i].p1.z()-env_min_z)*(roundFactor))/(cell_size*(roundFactor));
        int x2 = roundf((triangles[i].p2.x()-env_min_x)*(roundFactor))/(cell_size*(roundFactor));
        int y2 = roundf((triangles[i].p2.y()-env_min_y)*(roundFactor))/(cell_size*(roundFactor));
        int z2 = roundf((triangles[i].p2.z()-env_min_z)*(roundFactor))/(cell_size*(roundFactor));
        int x3 = roundf((triangles[i].p3.x()-env_min_x)*(roundFactor))/(cell_size*(roundFactor));
        int y3 = roundf((triangles[i].p3.y()-env_min_y)*(roundFactor))/(cell_size*(roundFactor));
        int z3 = roundf((triangles[i].p3.z()-env_min_z)*(roundFactor))/(cell_size*(roundFactor));

        int min_x = Utils::min_val(x1,x2,x3);
        int min_y = Utils::min_val(y1,y2,y3);
        int min_z = Utils::min_val(z1,z2,z3);

        int max_x = Utils::max_val(x1,x2,x3);
        int max_y = Utils::max_val(y1,y2,y3);
        int max_z = Utils::max_val(z1,z2,z3);

        //is the triangle right at the boundary between two cells (in any axis)?
        bool xLimit = Utils::eq(std::fmod(Utils::max_val(triangles[i][0].x(),triangles[i][1].x(),triangles[i][2].x())-env_min_x, cell_size),0)
            ||Utils::eq(std::fmod(Utils::max_val(triangles[i][0].x(),triangles[i][1].x(),triangles[i][2].x())-env_min_x, cell_size),cell_size);

        bool yLimit = Utils::eq(std::fmod(Utils::max_val(triangles[i][0].y(),triangles[i][1].y(),triangles[i][2].y())-env_min_y, cell_size),0)
            ||Utils::eq(std::fmod(Utils::max_val(triangles[i][0].y(),triangles[i][1].y(),triangles[i][2].y())-env_min_y, cell_size),cell_size);
            
        bool zLimit = Utils::eq(std::fmod(Utils::max_val(triangles[i][0].z(),triangles[i][1].z(),triangles[i][2].z())-env_min_z, cell_size),0)
            ||Utils::eq(std::fmod(Utils::max_val(triangles[i][0].z(),triangles[i][1].z(),triangles[i][2].z())-env_min_z, cell_size),cell_size);

        bool isParallel =Utils::isParallel(normals[i]);
        for (int row = min_x; row <= max_x && row < env[0].size(); row++)
        {
            for (int col = min_y; col <= max_y && col < env.size(); col++)
            {
                for (int height = min_z; height <= max_z && height < env[0][0].size(); height++)
                {
                    //check if the triangle goes through this cell
                    //special case for triangles that are parallel to the coordinate axes because the discretization can cause
                    //problems if they fall right on the boundary of two cells
                    if (
                        (isParallel && pointInTriangle(tf2::Vector3(row * cell_size + env_min_x+cell_size/2,
                                                        col * cell_size + env_min_y+cell_size/2,
                                                        height * cell_size + env_min_z+cell_size/2),
                                        tf2::Vector3(triangles[i][0].x(), triangles[i][0].y(), triangles[i][0].z()),
                                        tf2::Vector3(triangles[i][1].x(), triangles[i][1].y(), triangles[i][1].z()),
                                        tf2::Vector3(triangles[i][2].x(), triangles[i][2].y(), triangles[i][2].z())))
                    || 
                    triBoxOverlap(
                            tf2::Vector3(row * cell_size + env_min_x+cell_size/2,
                                col * cell_size + env_min_y+cell_size/2,
                                height * cell_size + env_min_z+cell_size/2),
                            tf2::Vector3(cell_size/2, cell_size/2, cell_size/2),
                            tf2::Vector3(triangles[i][0].x(), triangles[i][0].y(), triangles[i][0].z()),
                                    tf2::Vector3(triangles[i][1].x(), triangles[i][1].y(), triangles[i][1].z()),
                                    tf2::Vector3(triangles[i][2].x(), triangles[i][2].y(), triangles[i][2].z())))
                    {
                        mtx.lock();
                        env[col][row][height] = value_to_write;
                        if(value_to_write==cell_state::occupied){
                            //if the "limit" flags are activated, AND we are on the offending cells, 
                            //AND the cell has not previously marked as normally occupied by a different triangle
                            //AND the cells are not on the very limit of the environment, mark the cell as "edge" for later cleanup
                            bool limitOfproblematicTriangle=(xLimit&&row==max_x)||
                                (yLimit&&col==max_y)||
                                (zLimit&&height==max_z);

                            bool endOfTheEnvironment = (col>0 || col<env.size() ||
                                row>0 || row<env[0].size() ||
                                height>0 ||  height<env[0][0].size());

                            if( !endOfTheEnvironment &&
                                limitOfproblematicTriangle && 
                                env[col][row][height]!=cell_state::occupied){
                                    env[col][row][height]=cell_state::edge;
                            }
                        }
                        mtx.unlock();

                    }
                }
            }
        }

        //log progress
        if(i>numberOfProcessedTriangles+triangles.size()/10){
            mtx.lock();
            std::cout<<(100*i)/triangles.size()<<"%\n";
            numberOfProcessedTriangles=i;
            mtx.unlock();
        }
    }
}  

void Gaden_preprocessing::parse(const std::string& filename, cell_state value_to_write){
    
    bool ascii = isASCII(filename);
    
    std::vector<Triangle> triangles;
    std::vector<tf2::Vector3> normals;

    if(ascii){
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
        countfile.close();
        //each points[i] contains one the three vertices of triangle i
        triangles.resize(count);
        normals.resize(count);
        //let's read the data
        std::ifstream infile(filename.c_str());
        std::getline(infile, line);
        int i =0;
        while (line.find("endsolid")==std::string::npos)
        {
            while (line.find("facet normal") == std::string::npos){std::getline(infile, line);}
            size_t pos = line.find("facet");
            line.erase(0, pos + 12);
            float aux;
            std::stringstream ss(line);
            ss >> std::skipws >>  aux; 
            normals[i].setX( roundf(aux * roundFactor) / roundFactor );
            ss >> std::skipws >>  aux; 
            normals[i].setY( roundf(aux * roundFactor) / roundFactor );
            ss >> std::skipws >>  aux; 
            normals[i].setZ( roundf(aux * roundFactor) / roundFactor );
            std::getline(infile, line);

            for(int j=0;j<3;j++){
                std::getline(infile, line);
                size_t pos = line.find("vertex ");
                line.erase(0, pos + 7);
                std::stringstream ss(line);
                ss >> std::skipws >>  aux; 
                triangles[i][j].setX( roundf(aux * roundFactor) / roundFactor );
                ss >> std::skipws >>  aux; 
                triangles[i][j].setY( roundf(aux * roundFactor) / roundFactor );
                ss >> std::skipws >>  aux; 
                triangles[i][j].setZ( roundf(aux * roundFactor) / roundFactor );
            }
            i++;
            //skipping lines here makes checking for the end of the file more convenient
            std::getline(infile, line);
            std::getline(infile, line);
            while(std::getline(infile, line)&&line.length()==0);
        }
        infile.close();
    }
    else{
        std::ifstream infile(filename.c_str(), std::ios_base::binary);
        infile.seekg(80 * sizeof(uint8_t), std::ios_base::cur); //skip the header
        uint32_t num_triangles;
        infile.read((char*) &num_triangles, sizeof(uint32_t));
        triangles.resize(num_triangles);
        normals.resize(num_triangles);

        for(int i = 0; i < num_triangles; i++){
            infile.read((char*) &normals[i], 3 * sizeof(uint32_t)); //read the normal vector
            
            for(int j=0; j<3;j++)
            {
                std::array<float, 3> vec;
                infile.read((char*) &vec, 3 * sizeof(uint32_t)); //read the point
                triangles[i][j].setX(vec[0]);
                triangles[i][j].setY(vec[1]);
                triangles[i][j].setZ(vec[2]);
            }

            infile.seekg(sizeof(uint16_t), std::ios_base::cur); //skip the attribute data
        }
        infile.close();
    }
    
    //OK, we have read the data, let's do something with it
    occupy(triangles, normals, value_to_write);

}

bool Gaden_preprocessing::isASCII(const std::string& filename)
{
    bool ascii = false;
    if (FILE *file = fopen(filename.c_str(), "r"))
    {
        //File exists!, keep going!
        char buffer[6];
        fgets(buffer, 6, file);
        if(std::string(buffer).find("solid")!=std::string::npos)
            ascii=true;
        fclose(file);
    }else{
        std::cout<< "File " << filename << " does not exist\n";
        return false;
    }
    return ascii;
}

void Gaden_preprocessing::findDimensions(const std::string& filename){

    bool ascii = isASCII(filename);

    if(ascii){
        //let's read the data
        std::string line;
        std::ifstream infile(filename.c_str());
        std::getline(infile, line);
        int i =0;
        while (line.find("endsolid")==std::string::npos)
        {
            while (std::getline(infile, line) && line.find("outer loop") == std::string::npos);

            for(int j=0;j<3;j++){
                float x, y, z;
                std::getline(infile, line);
                size_t pos = line.find("vertex ");
                line.erase(0, pos + 7);
                std::stringstream ss(line);
                float aux;
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
        infile.close();
    }
    else{
        std::ifstream infile(filename.c_str(), std::ios_base::binary);
        infile.seekg(80 * sizeof(uint8_t), std::ios_base::cur); //skip the header
        uint32_t num_triangles;
        infile.read((char*) &num_triangles, sizeof(uint32_t));
        for(int i = 0; i < num_triangles; i++){
            infile.seekg(3 * sizeof(float), std::ios_base::cur); //skip the normal vector
            for(int j=0; j<3;j++){
                float x, y ,z;
                infile.read((char*) &x, sizeof(float));
                infile.read((char*) &y, sizeof(float));
                infile.read((char*) &z, sizeof(float));
                env_max_x = env_max_x>=x?env_max_x:x;
                env_max_y = env_max_y>=y?env_max_y:y;
                env_max_z = env_max_z>=z?env_max_z:z;
                env_min_x = env_min_x<=x?env_min_x:x;
                env_min_y = env_min_y<=y?env_min_y:y;
                env_min_z = env_min_z<=z?env_min_z:z;
            }
            
            infile.seekg(sizeof(uint16_t), std::ios_base::cur); //skip the attribute data
        }
    }
    
    std::cout<<"Dimensions are:\n"<<
    "x: ("<<env_min_x<<", "<<env_max_x<<")\n"<<
    "y: ("<<env_min_y<<", "<<env_max_y<<")\n"<<
    "z: ("<<env_min_z<<", "<<env_max_z<<")\n";


}


void Gaden_preprocessing::openFoam_to_gaden(const std::string& filename)
{

	//let's parse the file
	std::ifstream infile(filename.c_str());
	std::string line;

	//ignore the first line (column names)
	std::getline(infile, line);
    std::vector<double> U(env[0].size()*env.size()*env[0][0].size());
    std::vector<double> V(env[0].size()*env.size()*env[0][0].size());
    std::vector<double> W(env[0].size()*env.size()*env[0][0].size());
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
			x_idx = (int)roundf((v[3] - env_min_x) / cell_size*roundFactor)/roundFactor;
			y_idx = (int)roundf((v[4] - env_min_y) / cell_size*roundFactor)/roundFactor;
			z_idx = (int)roundf((v[5] - env_min_z) / cell_size*roundFactor)/roundFactor;
			U[ indexFrom3D(x_idx, y_idx,z_idx) ] = v[0];
			V[ indexFrom3D(x_idx, y_idx,z_idx) ] = v[1];
			W[ indexFrom3D(x_idx, y_idx,z_idx) ] = v[2];
		}
	}
    infile.close();
    printWind(U,V,W,filename);
}

void Gaden_preprocessing::fill(){
    float empty_point_x = declare_parameter<float>("empty_point_x", 0);
    float empty_point_y = declare_parameter<float>("empty_point_y", 0);
    float empty_point_z = declare_parameter<float>("empty_point_z", 0);
    
    int x =(empty_point_y-env_min_y)/cell_size;
    int z = (empty_point_z-env_min_z)/cell_size; 
    int y = (empty_point_x-env_min_x)/cell_size;

    cell_state new_value = cell_state::empty;
    cell_state value_to_overwrite = cell_state::non_initialized;

    std::queue<Eigen::Vector3i> q;
    q.push(Eigen::Vector3i(x, y, z));
    env[x][y][z]=new_value;
    while(!q.empty()){
        Eigen::Vector3i point = q.front();
        q.pop();
        if(compare_cell(point.x()+1, point.y(), point.z(), value_to_overwrite)){ // x+1, y, z
            env[point.x()+1][point.y()][point.z()]=new_value;
            q.push(Eigen::Vector3i(point.x()+1, point.y(), point.z()));
        }

        if(compare_cell(point.x()-1, point.y(), point.z(), value_to_overwrite)){ // x-1, y, z
            env[point.x()-1][point.y()][point.z()]=new_value;
            q.push(Eigen::Vector3i(point.x()-1, point.y(), point.z()));
        }

        if(compare_cell(point.x(), point.y()+1, point.z(), value_to_overwrite)){ // x, y+1, z
            env[point.x()][point.y()+1][point.z()]=new_value;
            q.push(Eigen::Vector3i(point.x(), point.y()+1, point.z()));
        }

        if(compare_cell(point.x(), point.y()-1, point.z(), value_to_overwrite)){ // x, y-1, z
            env[point.x()][point.y()-1][point.z()]=new_value;
            q.push(Eigen::Vector3i(point.x(), point.y()-1, point.z()));
        }

        if(compare_cell(point.x(), point.y(), point.z()+1, value_to_overwrite)){ // x, y, z+1
            env[point.x()][point.y()][point.z()+1]=new_value;
            q.push(Eigen::Vector3i(point.x(), point.y(), point.z()+1));
        }

        if(compare_cell(point.x(), point.y(), point.z()-1, value_to_overwrite)){ // x, y, z-1
            env[point.x()][point.y()][point.z()-1]=new_value;
            q.push(Eigen::Vector3i(point.x(), point.y(), point.z()-1));
        }
    }
}

void Gaden_preprocessing::clean(){
    for(int col=0;col<env.size();col++){
        for(int row=0;row<env[0].size();row++){
            for(int height=0;height<env[0][0].size();height++){
                
                if(env[col][row][height]==cell_state::edge){

                    if(compare_cell(col+1, row, height, cell_state::empty)||
                        compare_cell(col, row+1, height, cell_state::empty)||
                        compare_cell(col, row, height+1, cell_state::empty)||
                        (compare_cell(col+1, row+1, height, cell_state::empty)
                            &&env[col][row+1][height]==cell_state::edge
                            &&env[col+1][row][height]==cell_state::edge))
                    {
                        env[col][row][height]=cell_state::empty;
                    }else
                    {
                        env[col][row][height]=cell_state::occupied;
                    }
                    
                }
                
            }
        }
    }
}


void Gaden_preprocessing::generateOutput()
{
    std::string outputFolder = get_parameter_or<std::string>("output_path", "");
    printMap(
        boost::str(boost::format("%s/occupancy.pgm") % outputFolder.c_str()), 
        10, //scale 
        declare_parameter<bool>("block_outlets", false) );

    std::string worldFile;
    if(( worldFile = declare_parameter<std::string>("worldFile", "")) !="")
        changeWorldFile(worldFile);
    

    //output - path, occupancy vector, scale
    printEnv(boost::str(boost::format("%s/OccupancyGrid3D.csv") % outputFolder.c_str()), 1);
    printYaml(outputFolder);

}

void Gaden_preprocessing::processWind()
{
    bool uniformWind = declare_parameter<bool>("uniformWind", false);

    //path to the point cloud files with the wind data
    std::string windFileName = declare_parameter<std::string>("wind_files", "");
    int idx = 0;

    if(uniformWind){
        
        //let's parse the file
        std::ifstream infile(windFileName);
        std::string line;

        std::vector<double> U(env[0].size()*env.size()*env[0][0].size());
        std::vector<double> V(env[0].size()*env.size()*env[0][0].size());
        std::vector<double> W(env[0].size()*env.size()*env[0][0].size());
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
                        if(env[j][i][k]==cell_state::empty){
                           
                            U[ indexFrom3D(i, j, k) ] = v[0];
                            V[ indexFrom3D(i, j, k) ] = v[1];
                            W[ indexFrom3D(i, j, k) ] = v[2];
                        }
                    }
                }
            }
            infile.close();
            printWind(U,V,W, boost::str(boost::format("%s_%i.csv") % windFileName % idx).c_str());
            idx++;
        }
    }else{
        while (FILE *file = fopen(boost::str(boost::format("%s_%i.csv") % windFileName % idx).c_str(), "r"))
        {
            fclose(file);
            openFoam_to_gaden(boost::str(boost::format("%s_%i.csv") % windFileName % idx).c_str());
            idx++;
        }
    }
}

