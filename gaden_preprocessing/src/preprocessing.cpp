#include "preprocessing.hpp"
#include "TriangleBoxIntersection.hpp"
#include "gaden_common/Logging.h"
#include "gaden_common/Vector3.h"
#include <fmt/color.h>

#include <gaden_common/GadenVersion.h>
#include <gaden_common/Utils.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <queue>
#include <sstream>
#include <stdint.h>
#include <stdlib.h>
#include <string>
#include <yaml-cpp/yaml.h>

#ifdef GENERATE_COPPELIA_SCENE
#define SIM_REMOTEAPICLIENT_OBJECTS
#include "gaden_preprocessing/Gaden_preprocessing.h"
#include <RemoteAPIClient.h>
#endif

static constexpr float MAP_SCALE = 10;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<Gaden_preprocessing> node = std::make_shared<Gaden_preprocessing>();
    node->parseMainModels();
    if (!rclcpp::ok())
        return -1;
    node->parseOutletModels();

    // Mark all the empty cells reachable from the empty_point as aux_empty
    // the ones that cannot be reached will be marked as occupied when printing
    node->fill();

    node->processWind();
    node->generateOutput();

    GADEN_INFO_COLOR(fmt::terminal_color::blue, "Preprocessing done");
    std_msgs::msg::Bool b;
    b.data = true;
    node->jobDone_pub->publish(b);
    return 0;
}

void Gaden_preprocessing::parseMainModels()
{
    std::vector<std::string> stlModels = declare_parameter<std::vector<std::string>>("models", std::vector<std::string>{});

    if (stlModels.empty()) // try the old style, with numbered parameters instead of a single list
    {
        int i = 0;
        while (true)
        {
            std::string param_name = fmt::format("model_{}", i);
            std::string value = getParam<std::string>(shared_from_this(), param_name, "");
            if (value != "")
                stlModels.push_back(value);
            else
                break;
            i++;
        }
        if (i > 0)
            GADEN_WARN("Specifying models through numbered parameters is deprecated. You should use a single list parameter instead (see test_env "
                       "for examples)");
    }
    GADEN_INFO("Number of models: {}", stlModels.size());

    bool generateCoppeliaScene = getParam<bool>(shared_from_this(), "generateCoppeliaScene", false);
#ifdef GENERATE_COPPELIA_SCENE
    RemoteAPIClient client;
    if (generateCoppeliaScene)
    {
        RemoteAPIObject::sim sim = client.getObject().sim();
        sim.stopSimulation();
        while (sim.getSimulationState() != sim.simulation_stopped)
            ;
        float floor_height = getParam<float>(shared_from_this(), "floor_height", 0.0);
        sim.setObjectPosition(sim.getObject("/ResizableFloorLarge"), sim.handle_world, {0, 0, floor_height});
        sim.announceSceneContentChange();
    }
#else
    if (generateCoppeliaScene)
    {
        GADEN_ERROR("You are trying to generate a coppelia scene, but compiled gaden_preprocessing without coppelia support. Either "
                    "disable the request in the launch file or compile with coppelia support. \nYou can enable the generation in the "
                    "CMakeLists.txt file of the preprocessing package.");
        rclcpp::shutdown();
        return;
    }
#endif

    for (const std::string& model : stlModels)
    {
        findDimensions(model);
    }

    dimensions = ceil((env_max - env_min) / cell_size);
    env = std::vector<cell_state>(dimensions.x * dimensions.y * dimensions.z, cell_state::non_initialized);

    for (const std::string& model : stlModels)
    {
        GADEN_INFO("Parsing environment model: {}", model);
        parse(model, cell_state::occupied);
#ifdef GENERATE_COPPELIA_SCENE
        if (generateCoppeliaScene)
        {
            int result = -1;
            do
            {
                result = client.getObject().sim().importShape(0, model, 0, 0.0001f, 1);
            } while (result == -1);
        }
#endif
    }

    std::string outputFolder = getParam<std::string>(shared_from_this(), "output_path", "");

#ifdef GENERATE_COPPELIA_SCENE
    if (generateCoppeliaScene)
        client.getObject().sim().saveScene(fmt::format("{}/coppeliaScene.ttt", outputFolder));
#endif
}

void Gaden_preprocessing::parseOutletModels()
{
    std::vector<std::string> stlModels = declare_parameter<std::vector<std::string>>("outlets_models", std::vector<std::string>{});

    if (stlModels.empty()) // try the old style, with numbered parameters instead of a single list
    {
        int i = 0;
        while (true)
        {
            std::string param_name = fmt::format("outlet_model_{}", i);
            std::string value = getParam<std::string>(shared_from_this(), param_name, "");
            if (value != "")
                stlModels.push_back(value);
            else
                break;
            i++;
        }

        if (i > 0)
            GADEN_WARN("Specifying models through numbered parameters is deprecated. You should use a single list parameter instead (see test_env "
                       "for examples)");
    }
    GADEN_INFO("Number of outlet models: {}", stlModels.size());

    for (const std::string& model : stlModels)
    {
        GADEN_INFO("Parsing outlet model: {}", model);
        parse(model, cell_state::outlet);
    }
}

void Gaden_preprocessing::occupy(std::vector<Triangle>& triangles, const std::vector<gaden::Vector3>& normals, cell_state value_to_write)
{
    int numberOfProcessedTriangles = 0; // for logging, doesn't actually do anything
    std::mutex mtx;
    // Let's occupy the enviroment!
#pragma omp parallel for
    for (int i = 0; i < triangles.size(); i++)
    {
        // We try to find all the cells that some triangle goes through
        int x1 = (triangles[i].p1.x - env_min.x) / cell_size;
        int y1 = (triangles[i].p1.y - env_min.y) / cell_size;
        int z1 = (triangles[i].p1.z - env_min.z) / cell_size;
        int x2 = (triangles[i].p2.x - env_min.x) / cell_size;
        int y2 = (triangles[i].p2.y - env_min.y) / cell_size;
        int z2 = (triangles[i].p2.z - env_min.z) / cell_size;
        int x3 = (triangles[i].p3.x - env_min.x) / cell_size;
        int y3 = (triangles[i].p3.y - env_min.y) / cell_size;
        int z3 = (triangles[i].p3.z - env_min.z) / cell_size;

        // triangle Bounding Box
        int min_x = std::min({x1, x2, x3});
        int min_y = std::min({y1, y2, y3});
        int min_z = std::min({z1, z2, z3});

        int max_x = std::max({x1, x2, x3});
        int max_y = std::max({y1, y2, y3});
        int max_z = std::max({z1, z2, z3});

        bool isParallel = Utils::isParallel(normals[i]);
        for (int row = min_y; row <= max_y && row < dimensions.y; row++)
        {
            for (int col = min_x; col <= max_x && col < dimensions.x; col++)
            {
                for (int height = min_z; height <= max_z && height < dimensions.z; height++)
                {
                    // check if the triangle goes through this cell
                    // special case for triangles that are parallel to the coordinate axes because the discretization can cause
                    // problems if they fall right on the boundary of two cells
                    gaden::Vector3 cellCenter(
                        col * cell_size + env_min.x + cell_size / 2,
                        row * cell_size + env_min.y + cell_size / 2,
                        height * cell_size + env_min.z + cell_size / 2);
                    if ((isParallel && pointInTriangle(cellCenter, triangles[i])) || triBoxOverlap(cellCenter, triangles[i], cell_size * 0.5))
                    {
                        mtx.lock();
                        env[indexFrom3D(col, row, height)] = value_to_write;
                        mtx.unlock();
                    }
                }
            }
        }

        // log progress
        if (i > numberOfProcessedTriangles + triangles.size() / 10)
        {
            mtx.lock();
            GADEN_INFO("{}%", (int)((100 * i) / triangles.size()));
            numberOfProcessedTriangles = i;
            mtx.unlock();
        }
    }
}

void Gaden_preprocessing::parse(const std::string& filename, cell_state value_to_write)
{
    bool ascii = isASCII(filename);

    std::vector<Triangle> triangles;
    std::vector<gaden::Vector3> normals;

    if (ascii)
    {
        // first, we count how many triangles there are (we need to do this before reading the data
        //  to create a vector of the right size)
        std::ifstream countfile(filename.c_str());
        std::string line;
        int count = 0;

        while (std::getline(countfile, line))
        {
            if (line.find("facet normal") != std::string::npos)
            {
                count++;
            }
        }
        countfile.close();
        // each points[i] contains one the three vertices of triangle i
        triangles.resize(count);
        normals.resize(count);
        // let's read the data
        std::ifstream infile(filename.c_str());
        std::getline(infile, line);
        int i = 0;
        while (line.find("endsolid") == std::string::npos)
        {
            while (line.find("facet normal") == std::string::npos)
            {
                std::getline(infile, line);
            }
            size_t pos = line.find("facet");
            line.erase(0, pos + 12);
            float aux;
            std::stringstream ss(line);
            ss >> std::skipws >> aux;
            normals[i].x = aux;
            ss >> std::skipws >> aux;
            normals[i].y = aux;
            ss >> std::skipws >> aux;
            normals[i].z = aux;
            std::getline(infile, line);

            for (int j = 0; j < 3; j++)
            {
                std::getline(infile, line);
                size_t pos = line.find("vertex ");
                line.erase(0, pos + 7);
                std::stringstream ss(line);
                ss >> std::skipws >> aux;
                triangles[i][j].x = aux;
                ss >> std::skipws >> aux;
                triangles[i][j].y = aux;
                ss >> std::skipws >> aux;
                triangles[i][j].z = aux;
            }
            i++;
            // skipping lines here makes checking for the end of the file more convenient
            std::getline(infile, line);
            std::getline(infile, line);
            while (std::getline(infile, line) && line.length() == 0)
                ;
        }
        infile.close();
    }
    else
    {
        std::ifstream infile(filename.c_str(), std::ios_base::binary);
        infile.seekg(80 * sizeof(uint8_t), std::ios_base::cur); // skip the header
        uint32_t num_triangles;
        infile.read((char*)&num_triangles, sizeof(uint32_t));
        triangles.resize(num_triangles);
        normals.resize(num_triangles);

        for (int i = 0; i < num_triangles; i++)
        {
            infile.read((char*)&normals[i], 3 * sizeof(uint32_t)); // read the normal vector

            for (int j = 0; j < 3; j++)
            {
                std::array<float, 3> vec;
                infile.read((char*)&vec, 3 * sizeof(uint32_t)); // read the point
                triangles[i][j].x = (vec[0]);
                triangles[i][j].y = (vec[1]);
                triangles[i][j].z = (vec[2]);
            }

            infile.seekg(sizeof(uint16_t), std::ios_base::cur); // skip the attribute data
        }
        infile.close();
    }

    // OK, we have read the data, let's do something with it
    occupy(triangles, normals, value_to_write);
}

bool Gaden_preprocessing::isASCII(const std::string& filename)
{
    bool ascii = false;
    if (FILE* file = fopen(filename.c_str(), "r"))
    {
        // File exists!, keep going!
        char buffer[6];
        char* result = fgets(buffer, 6, file);

        if (std::string(buffer).find("solid") != std::string::npos)
            ascii = true;
        fclose(file);
    }
    else
    {
        GADEN_ERROR("File '{}' does not exist\n", filename.c_str());
        raise(SIGTRAP);
    }
    return ascii;
}

void Gaden_preprocessing::findDimensions(const std::string& filename)
{
    bool ascii = isASCII(filename);

    if (ascii)
    {
        // let's read the data
        std::string line;
        std::ifstream infile(filename.c_str());
        std::getline(infile, line);
        int i = 0;
        while (line.find("endsolid") == std::string::npos)
        {
            while (std::getline(infile, line) && line.find("outer loop") == std::string::npos)
                ;

            for (int j = 0; j < 3; j++)
            {
                float x, y, z;
                std::getline(infile, line);
                size_t pos = line.find("vertex ");
                line.erase(0, pos + 7);
                std::stringstream ss(line);
                float aux;
                ss >> std::skipws >> aux;
                x = aux;
                ss >> std::skipws >> aux;
                y = aux;
                ss >> std::skipws >> aux;
                z = aux;
                env_max.x = env_max.x >= x ? env_max.x : x;
                env_max.y = env_max.y >= y ? env_max.y : y;
                env_max.z = env_max.z >= z ? env_max.z : z;
                env_min.x = env_min.x <= x ? env_min.x : x;
                env_min.y = env_min.y <= y ? env_min.y : y;
                env_min.z = env_min.z <= z ? env_min.z : z;
            }
            i++;
            // skipping three lines here makes checking for the end of the file more convenient
            std::getline(infile, line);
            std::getline(infile, line);
            while (std::getline(infile, line) && line.length() == 0)
                ;
        }
        infile.close();
    }
    else
    {
        std::ifstream infile(filename.c_str(), std::ios_base::binary);
        infile.seekg(80 * sizeof(uint8_t), std::ios_base::cur); // skip the header
        uint32_t num_triangles;
        infile.read((char*)&num_triangles, sizeof(uint32_t));
        for (int i = 0; i < num_triangles; i++)
        {
            infile.seekg(3 * sizeof(float), std::ios_base::cur); // skip the normal vector
            for (int j = 0; j < 3; j++)
            {
                float x, y, z;
                infile.read((char*)&x, sizeof(float));
                infile.read((char*)&y, sizeof(float));
                infile.read((char*)&z, sizeof(float));
                env_max.x = env_max.x >= x ? env_max.x : x;
                env_max.y = env_max.y >= y ? env_max.y : y;
                env_max.z = env_max.z >= z ? env_max.z : z;
                env_min.x = env_min.x <= x ? env_min.x : x;
                env_min.y = env_min.y <= y ? env_min.y : y;
                env_min.z = env_min.z <= z ? env_min.z : z;
            }

            infile.seekg(sizeof(uint16_t), std::ios_base::cur); // skip the attribute data
        }
    }

    GADEN_INFO("Dimensions are:\n"
               "	x : ({}, {})\n"
               "	y : ({}, {})\n"
               "	z : ({}, {})\n",
               env_min.x, env_max.x, env_min.y, env_max.y, env_min.z, env_max.z);
}

void Gaden_preprocessing::fill()
{
    // essentially a flood fill algorithm
    // start from a point specified by a ros parameter, and replace any uninitialized cells you find with free ones
    // occupied cells block the propagation, so the only cells that will remain uninitialized at the end are the ones that are unreachable from the seed point
    // i.e. inside of obstacles
    std::queue<gaden::Vector3i> q;
    {
        gaden::Vector3 empty_point =
            {
                getParam<float>(shared_from_this(), "empty_point_x", 0),
                getParam<float>(shared_from_this(), "empty_point_y", 0),
                getParam<float>(shared_from_this(), "empty_point_z", 0)};

        gaden::Vector3i indices = (empty_point - env_min) / cell_size;

        q.emplace(indices);
        env[indexFrom3D(indices)] = cell_state::empty;
    }

    while (!q.empty())
    {
        gaden::Vector3i oldPoint = q.front();
        q.pop();

        // if oldPoint+offset is non_initialized, set it to free and add its indices to the queue to keep propagating
        auto compareAndAdd = [&](gaden::Vector3i offset)
        {
            gaden::Vector3i currentPoint = oldPoint + offset;
            if (compare_cell(currentPoint, cell_state::non_initialized))
            {
                env[indexFrom3D(currentPoint)] = cell_state::empty;
                q.emplace(currentPoint);
            }
        };

        compareAndAdd({1, 0, 0});
        compareAndAdd({-1, 0, 0});

        compareAndAdd({0, 1, 0});
        compareAndAdd({0, -1, 0});

        compareAndAdd({0, 0, 1});
        compareAndAdd({0, 0, -1});
    }
}

// WIND
//---------------------------------

void Gaden_preprocessing::processWind()
{
    bool uniformWind = getParam<bool>(shared_from_this(), "uniformWind", false);

    // path to the point cloud files with the wind data
    std::string windFileName = getParam<std::string>(shared_from_this(), "wind_files", "");
    int idx = 0;

    if (uniformWind)
    {
        // let's parse the file
        std::ifstream infile(windFileName);
        std::string line;

        std::vector<gaden::Vector3> wind(env.size());
        while (std::getline(infile, line))
        {
            std::vector<double> v;
            for (int i = 0; i < 3; i++)
            {
                size_t pos = line.find(",");
                v.push_back(atof(line.substr(0, pos).c_str()));
                line.erase(0, pos + 1);
            }

            for (int i = 0; i < dimensions.x; i++)
            {
                for (int j = 0; j < dimensions.y; j++)
                {
                    for (int k = 0; k < dimensions.z; k++)
                    {
                        if (env[indexFrom3D(i, j, k)] == cell_state::empty)
                        {
                            wind[indexFrom3D(i, j, k)].x = v[0];
                            wind[indexFrom3D(i, j, k)].y = v[1];
                            wind[indexFrom3D(i, j, k)].z = v[2];
                        }
                    }
                }
            }
            infile.close();
            printWindFiles(wind, fmt::format("{}_{}.csv", windFileName, idx));
            idx++;
        }
    }
    else
    {
        // check that there is at least one wind file
        {
            std::string filename = fmt::format("{}_0.csv", windFileName);
            if (!std::filesystem::exists(filename))
            {
                GADEN_WARN("File {} does not exist", filename.c_str());
                return;
            }
        }

        while (std::filesystem::exists(fmt::format("{}_{}.csv", windFileName, idx)))
        {
            openFoam_to_gaden(fmt::format("{}_{}.csv", windFileName, idx));
            idx++;
        }
    }
}

void Gaden_preprocessing::openFoam_to_gaden(const std::string& filename)
{
    // let's parse the file
    std::ifstream infile(filename.c_str());
    std::string line;
    struct ParsedLine
    {
        float point[3];
        float windVector[3];
    };
    ParsedLine parsedLine;

    // Depending on the verion of Paraview used to export the file, lines might be (Point, vector) OR (vector, Point)
    // so we need to check the header before we know where to put what
    float* firstPartOfLine;
    float* secondPartOfLine;
    {
        std::getline(infile, line);
        size_t pos = line.find(",");
        std::string firstElement = line.substr(0, pos);

        if (firstElement.find("Points") != std::string::npos)
        {
            firstPartOfLine = parsedLine.point;
            secondPartOfLine = parsedLine.windVector;
        }
        else
        {
            firstPartOfLine = parsedLine.windVector;
            secondPartOfLine = parsedLine.point;
        }
    }

    std::vector<gaden::Vector3> wind(env.size(), gaden::Vector3(0, 0, 0));

    int x_idx = 0;
    int y_idx = 0;
    int z_idx = 0;
    while (std::getline(infile, line))
    {
        if (line.length() != 0)
        {
            for (int i = 0; i < 3; i++)
            {
                size_t pos = line.find(",");
                firstPartOfLine[i] = atof(line.substr(0, pos).c_str());
                line.erase(0, pos + 1);
            }

            for (int i = 0; i < 3; i++)
            {
                size_t pos = line.find(",");
                secondPartOfLine[i] = atof(line.substr(0, pos).c_str());
                line.erase(0, pos + 1);
            }

            // assign each of the points we have information about to the nearest cell
            x_idx = (parsedLine.point[0] - env_min.x) / cell_size;
            y_idx = (parsedLine.point[1] - env_min.y) / cell_size;
            z_idx = (parsedLine.point[2] - env_min.z) / cell_size;

            size_t index3D = indexFrom3D(x_idx, y_idx, z_idx);
            wind[index3D].x = parsedLine.windVector[0];
            wind[index3D].y = parsedLine.windVector[1];
            wind[index3D].z = parsedLine.windVector[2];
        }
    }

    infile.close();
    printWindFiles(wind, filename);
}

/// OUTPUT
//-------------------------------------

void Gaden_preprocessing::generateOutput()
{
    std::string outputFolder = get_parameter_or<std::string>("output_path", "");
    if (!std::filesystem::exists(outputFolder))
    {
        GADEN_ERROR("Output folder '{}' does not exist!", outputFolder);
        return;
    }

    GADEN_INFO_COLOR(fmt::terminal_color::blue, "Writing output to folder '{}'", outputFolder);
    printOccupancyMap(fmt::format("{}/occupancy.pgm", outputFolder), getParam<bool>(shared_from_this(), "block_outlets", false));
    printOccupancyYaml(outputFolder);

    std::string worldFile;
    if ((worldFile = getParam<std::string>(shared_from_this(), "worldFile", "")) != "")
        changeStageWorldFile(worldFile);

    printBasicSimYaml(outputFolder);

    // output - path, occupancy vector, scale
    printGadenEnvFile(fmt::format("{}/OccupancyGrid3D.csv", outputFolder));
}

void Gaden_preprocessing::changeStageWorldFile(const std::string& filename)
{
    std::ifstream input(filename);
    std::stringstream ss;
    std::string line;

    float floor_height = getParam<float>(shared_from_this(), "floor_height", 0.0);
    while (getline(input, line))
    {
        if (line.substr(0, 8) == "floorMap")
        {
            // ignore the floorMap bit, we are replacing it entirely
            while (getline(input, line) && line != ")")
            {
            }

            ss << "floorMap                     # load an environment bitmap\n"
               << "(\n"
               << "  name \"SimulatedMap\"\n"
               << "  bitmap \"../../occupancy.pgm\"\n"
               << "  size [" << (env_max.x - env_min.x) << " " << (env_max.y - env_min.y) << " " << (env_max.z - env_min.z) << "]           #m \n"
               << "  pose [" << (env_max.x - env_min.x) / 2 + env_min.x << " " << (env_max.y - env_min.y) / 2 + env_min.y << " " << floor_height
               << " 0]    #Coordinates (m) of the Center of the image_map\n"
               << ")\n";
        }
        else
        {
            ss << line << "\n";
        }
    }
    input.close();
    std::ofstream out(filename);
    out << ss.rdbuf();
    out.close();
}

void Gaden_preprocessing::printOccupancyMap(std::string_view filename, bool block_outlets)
{
    std::ofstream outfile(filename.data());
    outfile << "P2\n"
            << dimensions.x << " " << dimensions.y << "\n"
            << "1\n";
    // things are repeated to scale them up (the image is too small!)

    float floor_height = getParam<float>(shared_from_this(), "floor_height", 0);
    int height = (floor_height - env_min.z) / cell_size; // a xy slice of the 3D environment is used as a geometric map for navigation
    if (height >= dimensions.z)
    {
        GADEN_ERROR("Cannot print the occupancy map at height {} -- the environment only gets to height {}", floor_height, env_max.z);
        return;
    }
    for (int row = dimensions.y - 1; row >= 0; row--)
    {
        for (int col = 0; col < dimensions.x; col++)
        {
            auto& cell = env[indexFrom3D(col, row, height)];
            bool outletTerm = cell == cell_state::outlet && !block_outlets;
            outfile << (cell == cell_state::empty || outletTerm ? 1 : 0) << " ";
            // outfile << cell << " ";
        }
        outfile << "\n";
    }
    outfile.close();
}

void Gaden_preprocessing::printOccupancyYaml(std::string_view outputFolder)
{
    std::ofstream file(fmt::format("{}/occupancy.yaml", outputFolder));
    YAML::Emitter yaml;
    yaml.SetDoublePrecision(3);
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "image" << YAML::Value << "occupancy.pgm";
    yaml << YAML::Key << "resolution" << YAML::Value << cell_size;

    float floor_height = getParam<float>(shared_from_this(), "floor_height", 0.0);
    yaml << YAML::Key << "origin" << YAML::Value << YAML::Flow << std::vector<float>{env_min.x, env_min.y, 0.0}; // the third component is yaw, not Z!
    yaml << YAML::Key << "occupied_thresh" << YAML::Value << 0.9;
    yaml << YAML::Key << "free_thresh" << YAML::Value << 0.1;
    yaml << YAML::Key << "negate" << YAML::Value << 0;

    yaml << YAML::EndMap;
    file << yaml.c_str();
    file.close();
}

void Gaden_preprocessing::printBasicSimYaml(std::string_view outputFolder)
{
    std::ofstream file(fmt::format("{}/BasicSimScene.yaml", outputFolder));
    YAML::Emitter yaml;
    yaml.SetDoublePrecision(2);
    yaml << YAML::BeginMap;
    yaml << YAML::Key << "map" << YAML::Value << "occupancy.yaml";
    yaml << YAML::Key << "robots" << YAML::BeginSeq;

    // robot entry
    {
        yaml << YAML::BeginMap;
        yaml << YAML::Key << "name" << YAML::Value << "PioneerP3DX";
        yaml << YAML::Key << "radius" << YAML::Value << 0.25;

        float floor_height = getParam<float>(shared_from_this(), "floor_height", 0.0);
        std::vector<float> startingPoint{getParam<float>(shared_from_this(), "empty_point_x", 0),
                                         getParam<float>(shared_from_this(), "empty_point_y", 0), floor_height};
        yaml << YAML::Key << "position" << YAML::Value << YAML::Flow << startingPoint;
        yaml << YAML::Key << "angle" << YAML::Value << 0.0 << YAML::Comment("in radians");
        yaml << YAML::Key << "sensors" << YAML::BeginSeq;

        // sensor entry
        {
            yaml << YAML::BeginMap;
            yaml << YAML::Key << "type" << YAML::Value << "laser";
            yaml << YAML::Key << "name" << YAML::Value << "laser_scanner";
            yaml << YAML::Key << "minAngleRad" << YAML::Value << -2.2;
            yaml << YAML::Key << "maxAngleRad" << YAML::Value << 2.2;
            yaml << YAML::Key << "angleResolutionRad" << YAML::Value << 0.07;
            yaml << YAML::Key << "minDistance" << YAML::Value << 0.1;
            yaml << YAML::Key << "maxDistance" << YAML::Value << 4.0;
            yaml << YAML::EndMap;
        }
        yaml << YAML::EndSeq;

        yaml << YAML::EndMap;
    }
    yaml << YAML::EndSeq;

    yaml << YAML::EndMap;

    file << yaml.c_str();
    file.close();
}

void Gaden_preprocessing::printGadenEnvFile(std::string_view filename)
{
    std::ofstream outfile(filename.data());

    outfile << "#env_min(m) " << env_min.x << " " << env_min.y << " " << env_min.z << "\n";
    outfile << "#env_max(m) " << env_max.x << " " << env_max.y << " " << env_max.z << "\n";
    outfile << "#num_cells " << dimensions.x << " " << dimensions.y << " " << dimensions.z << "\n";
    outfile << "#cell_size(m) " << cell_size << "\n";
    // things are repeated to scale them up (the image is too small!)
    for (int height = 0; height < dimensions.z; height++)
    {
        for (int col = 0; col < dimensions.x; col++)
        {
            for (int row = 0; row < dimensions.y; row++)
            {
                outfile << (env[indexFrom3D(col, row, height)] == cell_state::empty ? 0
                                                                                    : (env[indexFrom3D(col, row, height)] == cell_state::outlet ? 2 : 1))
                        << " ";
            }
            outfile << "\n";
        }
        outfile << ";\n";
    }
    outfile.close();
}

void Gaden_preprocessing::printWindFiles(const std::vector<gaden::Vector3>& wind, std::string_view filename)
{
    std::ofstream outputFile(fmt::format("{}_gaden", filename));

    outputFile.write((char*)&gaden::version_major, sizeof(int));
    outputFile.write((char*)&gaden::version_minor, sizeof(int));

    outputFile.write((char*)wind.data(), sizeof(gaden::Vector3) * wind.size());

    outputFile.close();
}

// AUX CHECKS
//-----------------------------------------

bool Gaden_preprocessing::compare_cell(gaden::Vector3i pos, cell_state value)
{
    if (pos.x < 0 || pos.x >= dimensions.x || pos.y < 0 || pos.y >= dimensions.y || pos.z < 0 || pos.z >= dimensions.z)
    {
        return false;
    }
    else
    {
        return env[indexFrom3D(pos)] == value;
    }
}

std::array<gaden::Vector3, 9> Gaden_preprocessing::cubePoints(const gaden::Vector3& query_point)
{
    std::array<gaden::Vector3, 9> points;
    points[0] = (query_point);
    points[1] = (gaden::Vector3(query_point.x - cell_size / 2, query_point.y - cell_size / 2, query_point.z - cell_size / 2));
    points[2] = (gaden::Vector3(query_point.x - cell_size / 2, query_point.y - cell_size / 2, query_point.z + cell_size / 2));
    points[3] = (gaden::Vector3(query_point.x - cell_size / 2, query_point.y + cell_size / 2, query_point.z - cell_size / 2));
    points[4] = (gaden::Vector3(query_point.x - cell_size / 2, query_point.y + cell_size / 2, query_point.z + cell_size / 2));
    points[5] = (gaden::Vector3(query_point.x + cell_size / 2, query_point.y - cell_size / 2, query_point.z - cell_size / 2));
    points[6] = (gaden::Vector3(query_point.x + cell_size / 2, query_point.y - cell_size / 2, query_point.z + cell_size / 2));
    points[7] = (gaden::Vector3(query_point.x + cell_size / 2, query_point.y + cell_size / 2, query_point.z - cell_size / 2));
    points[8] = (gaden::Vector3(query_point.x + cell_size / 2, query_point.y + cell_size / 2, query_point.z + cell_size / 2));
    return points;
}

bool Gaden_preprocessing::pointInTriangle(const gaden::Vector3& query_point, Triangle& triangle)
{
    // u=P2−P1
    gaden::Vector3 u = triangle[1] - triangle[0];
    // v=P3−P1
    gaden::Vector3 v = triangle[2] - triangle[0];
    // n=u×v
    gaden::Vector3 n = gaden::cross(u, v);
    bool anyProyectionInTriangle = false;
    std::array<gaden::Vector3, 9> cube = cubePoints(query_point);
    for (const gaden::Vector3& vec : cube)
    {
        // w=P−P1
        gaden::Vector3 w = vec - triangle[0];
        // Barycentric coordinates of the projection P′of P onto T:
        // γ=[(u×w)⋅n]/n²
        float gamma = gaden::dot(gaden::cross(u, w), n) / gaden::dot(n, n);
        // β=[(w×v)⋅n]/n²
        float beta = gaden::dot(gaden::cross(w, v), n) / gaden::dot(n, n);
        float alpha = 1 - gamma - beta;
        // The point P′ lies inside T if:
        bool proyectionInTriangle = ((0 <= alpha) && (alpha <= 1) && (0 <= beta) && (beta <= 1) && (0 <= gamma) && (gamma <= 1));
        anyProyectionInTriangle = anyProyectionInTriangle || proyectionInTriangle;
    }

    n = gaden::normalized(n);

    // we consider that the triangle goes through the cell if the proyection of the center
    // is inside the triangle AND the plane of the triangle intersects the cube of the cell

    return anyProyectionInTriangle;
}