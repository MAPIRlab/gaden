#define GADEN_LOGGER_ID "Gaden-Preprocessing"

#include "preprocessing.hpp"
#include "gaden/EnvironmentConfiguration.hpp"
#include "gaden/Preprocessing.hpp"
#include "gaden/core/Logging.hpp"
#include "gaden/internal/PathUtils.hpp"
#include "gaden_common/Utils.hpp"
using namespace gaden;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<Gaden_preprocessing> node = std::make_shared<Gaden_preprocessing>();
    node->Run();
    return 0;
}

void Gaden_preprocessing::Run()
{
    // if there is a gaden project directory, we will just parse those files instead of reading everything from ROS params
    std::filesystem::path projectPath = GadenUtils::getParam<std::string>(shared_from_this(), "projectPath", "");
    if (std::filesystem::exists(projectPath))
    {
        gadenProject.emplace(projectPath);
        GADEN_CHECK_RESULT(gadenProject->ReadDirectory());
    }

    // These variables can be passed through ROS parameters (as always) or read from the gaden project files (new)
    //--------------------------------------------------------
    float cellSize;
    Vector3 emptyPoint;
    std::filesystem::path outputFolder;
    EnvironmentConfiguration config;
    std::vector<std::filesystem::path> models;
    std::vector<std::filesystem::path> outletModels;

    if (gadenProject)
    {
        cellSize = gadenProject->cellSize;
        emptyPoint = gadenProject->emptyPoint;
        outputFolder = projectPath;
        models = EnvironmentConfigMetadata::GetPaths(gadenProject->envModels);
        outletModels = EnvironmentConfigMetadata::GetPaths(gadenProject->outletModels);
    }
    else
    {
        GadenUtils::OldProjectWarning();
        cellSize = GadenUtils::getParam<float>(shared_from_this(), "cell_size", 0.1);
        emptyPoint = {
            GadenUtils::getParam<float>(shared_from_this(), "empty_point_x", 0),
            GadenUtils::getParam<float>(shared_from_this(), "empty_point_y", 0),
            GadenUtils::getParam<float>(shared_from_this(), "empty_point_z", 0)};
        outputFolder = GadenUtils::getParam<std::string>(shared_from_this(), "output_path", "");
        models = GetModels("model");
        outletModels = GetModels("outlets_model");
    }
    //--------------------------------------------------------

    GADEN_INFO("Parsing geometry files...");
    config.environment = Preprocessing::ParseSTLModels(models, outletModels, cellSize, emptyPoint);
    GADEN_INFO("Parsing wind files...");
    config.windSequence = gadenProject ? GetWindSequenceProject(config.environment) : GetWindSequenceROSParams(config.environment);

    // generate output
    GADEN_INFO_COLOR(fmt::terminal_color::blue, "Writing output to folder '{}'", outputFolder);


    // this is needed for compatibility with old launch files, which do not respect the structure of gaden projects
    // in those, store the wind data back into the folder with the unprocessed files
    if (gadenProject)
        config.WriteToDirectory(outputFolder);
    else
    {
        config.environment.WriteToFile(outputFolder / "OccupancyGrid3D.csv");
        std::string windFileName = GadenUtils::getParam<std::string>(shared_from_this(), "wind_files", "");
        std::filesystem::create_directories(windFileName);
        config.windSequence.WriteToFiles(windFileName, "wind_iteration");
    }

    float floorHeight = GadenUtils::getParam<float>(shared_from_this(), "floor_height", 0.0);
    config.environment.Write2DSlicePGM(outputFolder / "occupancy.pgm",
                                       floorHeight,
                                       GadenUtils::getParam<bool>(shared_from_this(), "block_outlets", false));

    config.environment.WriteROSOccupancyYAML(outputFolder / "occupancy.yaml", floorHeight);
    config.environment.printBasicSimYaml(outputFolder / "BasicSimScene.yaml", gaden::Vector3(emptyPoint.x, emptyPoint.y, floorHeight));

    // notify we are done!
    GADEN_INFO_COLOR(fmt::terminal_color::blue, "Preprocessing done");
    std_msgs::msg::Bool b;
    b.data = true;
    jobDone_pub->publish(b);
}

std::vector<std::filesystem::path> Gaden_preprocessing::GetModels(const std::string& parameter_name)
{
    std::vector<std::string> stlModels = declare_parameter<std::vector<std::string>>(fmt::format("{}s", parameter_name.data()), std::vector<std::string>{});

    // delete special lines (starting with '!') which are used to specify colors for the environment node
    for (int i = stlModels.size() - 1; i >= 0; i--)
    {
        if (stlModels.at(i).at(0) == '!')
            stlModels.erase(stlModels.begin() + i);
    }

    if (stlModels.empty()) // try the old style, with numbered parameters instead of a single list
    {
        int i = 0;
        while (true)
        {
            std::string numbered_param_name = fmt::format("{}_{}", parameter_name, i);
            std::string value = GadenUtils::getParam<std::string>(shared_from_this(), numbered_param_name, "");
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
    GADEN_INFO("Number of {}s: {}", parameter_name, stlModels.size());

    return gaden::paths::AsPaths(stlModels);
}

WindSequence Gaden_preprocessing::GetWindSequenceROSParams(const gaden::Environment& env)
{
    bool uniformWind = GadenUtils::getParam<bool>(shared_from_this(), "uniformWind", false);

    // path to the point cloud files with the wind data
    std::string windFileName = GadenUtils::getParam<std::string>(shared_from_this(), "wind_files", "");

    if (uniformWind)
    {
        // the file just contains a list of vectors, where each vector is shared by all the cells in a particular timestep
        return WindSequence::CreateUniformWind(windFileName, env.numCells());
    }
    else
    {
        std::vector<std::filesystem::path> paths = GadenUtils::GetWindFiles([](std::string const& path, size_t idx)
                                                                            {
                                                                                return fmt::format("{}_{}.csv", path, idx);
                                                                            },
                                                                            windFileName);
        return Preprocessing::ParseOpenFoamVectorCloud(paths, env, {});
    }
}

gaden::WindSequence Gaden_preprocessing::GetWindSequenceProject(const gaden::Environment& env)
{
    if (gadenProject->uniformWind)
        return WindSequence::CreateUniformWind(gadenProject->GetWindFiles()[0], env.numCells());
    else
        return Preprocessing::ParseOpenFoamVectorCloud(gadenProject->GetWindFiles(), env, {});
}
