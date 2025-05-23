#define GADEN_LOGGER_ID "Gaden-Preprocessing"

#include "preprocessing.hpp"
#include "gaden/EnvironmentConfiguration.hpp"
#include "gaden/Preprocessing.hpp"
#include "gaden/core/Logging.hpp"
#include "gaden_common/Utils.hpp"
#include <fstream>
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
    float cellSize = GadenUtils::getParam<float>(shared_from_this(), "cell_size", 0.1);
    Vector3 emptyPoint = {
        GadenUtils::getParam<float>(shared_from_this(), "empty_point_x", 0),
        GadenUtils::getParam<float>(shared_from_this(), "empty_point_y", 0),
        GadenUtils::getParam<float>(shared_from_this(), "empty_point_z", 0)};

    auto models = GetModels("model");
    auto outletModels = GetModels("outlet_model");
    GADEN_INFO("Parsing geometry files...");
    Environment env = Preprocessing::ParseSTLModels(models, outletModels, cellSize, emptyPoint);
    GADEN_INFO("Parsing wind files...");
    WindSequence sequence = GetWindSequence(env);

    // generate output
    std::filesystem::path outputFolder = GadenUtils::getParam<std::string>(shared_from_this(), "output_path", "");

    GADEN_INFO_COLOR(fmt::terminal_color::blue, "Writing output to folder '{}'", outputFolder);
    EnvironmentConfiguration config{.environment = env,
                                    .windSequence = sequence,
                                    .path = outputFolder};
    config.WriteToDirectory();

    float floorHeight = GadenUtils::getParam<float>(shared_from_this(), "floor_height", 0.0);
    env.Write2DSlicePGM(outputFolder / "occupancy.pgm",
                        floorHeight,
                        GadenUtils::getParam<bool>(shared_from_this(), "block_outlets", false));

    env.WriteROSOccupancyYAML(outputFolder / "occupancy.yaml", floorHeight);
    env.printBasicSimYaml(outputFolder / "BasicSimScene.yaml", emptyPoint);

    // notify we are done!
    GADEN_INFO_COLOR(fmt::terminal_color::blue, "Preprocessing done");
    std_msgs::msg::Bool b;
    b.data = true;
    jobDone_pub->publish(b);
}

std::vector<std::filesystem::path> Gaden_preprocessing::GetModels(const std::string& parameter_name)
{
    std::vector<std::string> stlModels = declare_parameter<std::vector<std::string>>(fmt::format("{}s", parameter_name.data()), std::vector<std::string>{});

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

    return GadenUtils::AsPaths(stlModels);
}

WindSequence Gaden_preprocessing::GetWindSequence(const gaden::Environment& env)
{
    bool uniformWind = GadenUtils::getParam<bool>(shared_from_this(), "uniformWind", false);

    // path to the point cloud files with the wind data
    std::string windFileName = GadenUtils::getParam<std::string>(shared_from_this(), "wind_files", "");

    if (uniformWind)
    {
        // the file just containes a list of vectors, where each vector is shared by all the cells in a particular timestep

        std::vector<std::vector<gaden::Vector3>> windMaps;
        std::ifstream infile(windFileName);
        std::string line;

        std::vector<gaden::Vector3> timestep(env.numCells(), Vector3{0, 0, 0});
        while (std::getline(infile, line))
        {
            Vector3 v;
            for (int i = 0; i < 3; i++)
            {
                size_t pos = line.find(",");
                v[i] = (atof(line.substr(0, pos).c_str()));
                line.erase(0, pos + 1);
            }

            for (size_t i = 0; i < timestep.size(); i++)
                if (env.cells[i] == Environment::CellState::Free)
                    timestep[i] = v;

            windMaps.push_back(timestep);
        }
        infile.close();

        WindSequence seq;
        seq.Initialize(windMaps, env.numCells(), {});
        return seq;
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
