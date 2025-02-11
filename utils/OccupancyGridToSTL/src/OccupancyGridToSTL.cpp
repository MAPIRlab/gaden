#include <fmt/format.h>
#include <fstream>
#include <gaden_common/ReadEnvironment.h>
#include <spdlog/spdlog.h>

struct Triangle
{
    gaden::Vector3 normal;
    gaden::Vector3 vertices[3];
};

int main(int argc, char** argv)
{
    if (argc < 3 || argc > 5)
    {
        spdlog::error("Wrong number of arguments. Correct format is:\n"
                      "OccupancyGridToSTL <input path> <output path> [(bool) innerVolume = true] [(bool) ascii = false]");
        return -1;
    }

    gaden::Environment environment;
    gaden::ReadResult result = gaden::readEnvFile(argv[1], environment);

    if (result != gaden::ReadResult::OK)
    {
        spdlog::error("Could not open input file {}", argv[1]);
        return -1;
    }

    // Generate the triangles
    //-----------------------
    bool generateInnerVolume = true;
    if (argc >= 4)
        generateInnerVolume = strcmp(argv[3], "true") == 0 || strcmp(argv[3], "True") == 0;

    gaden::CellState targetState = generateInnerVolume ? gaden::CellState::Obstacle : gaden::CellState::Free;
    std::vector<Triangle> triangles;
    for (int z = 0; z < environment.description.dimensions.z; z++)
    {
        for (int x = 0; x < environment.description.dimensions.x; x++)
        {
            for (int y = 0; y < environment.description.dimensions.y; y++)
            {
                if (environment.at(x, y, z) == targetState)
                    continue;

                gaden::Vector3 center = environment.coordsOfCellCenter({x, y, z});
                float offset = environment.description.cell_size * 0.5f;
                // Z
                {
                    if (z - 1 < 0 || environment.at(x, y, z - 1) == targetState)
                    {
                        triangles.emplace_back();
                        triangles.back().normal = {0, 0, -1};
                        triangles.back().vertices[0] = center + offset * gaden::Vector3{-1, -1, -1};
                        triangles.back().vertices[1] = center + offset * gaden::Vector3{-1, +1, -1};
                        triangles.back().vertices[2] = center + offset * gaden::Vector3{+1, +1, -1};

                        triangles.emplace_back();
                        triangles.back().normal = {0, 0, -1};
                        triangles.back().vertices[0] = center + offset * gaden::Vector3{+1, +1, -1};
                        triangles.back().vertices[1] = center + offset * gaden::Vector3{+1, -1, -1};
                        triangles.back().vertices[2] = center + offset * gaden::Vector3{-1, -1, -1};
                    }

                    if (z + 1 >= environment.description.dimensions.z || environment.at(x, y, z + 1) == targetState)
                    {
                        triangles.emplace_back();
                        triangles.back().normal = {0, 0, 1};
                        triangles.back().vertices[0] = center + offset * gaden::Vector3{-1, -1, +1};
                        triangles.back().vertices[1] = center + offset * gaden::Vector3{+1, +1, +1};
                        triangles.back().vertices[2] = center + offset * gaden::Vector3{-1, +1, +1};

                        triangles.emplace_back();
                        triangles.back().normal = {0, 0, 1};
                        triangles.back().vertices[0] = center + offset * gaden::Vector3{+1, +1, +1};
                        triangles.back().vertices[1] = center + offset * gaden::Vector3{-1, -1, +1};
                        triangles.back().vertices[2] = center + offset * gaden::Vector3{+1, -1, +1};
                    }
                }

                // X
                {
                    if (x - 1 < 0 || environment.at(x - 1, y, z) == targetState)
                    {
                        triangles.emplace_back();
                        triangles.back().normal = {-1, 0, 0};
                        triangles.back().vertices[0] = center + offset * gaden::Vector3{-1, -1, -1};
                        triangles.back().vertices[1] = center + offset * gaden::Vector3{-1, -1, +1};
                        triangles.back().vertices[2] = center + offset * gaden::Vector3{-1, +1, +1};

                        triangles.emplace_back();
                        triangles.back().normal = {-1, 0, 0};
                        triangles.back().vertices[0] = center + offset * gaden::Vector3{-1, +1, +1};
                        triangles.back().vertices[1] = center + offset * gaden::Vector3{-1, +1, -1};
                        triangles.back().vertices[2] = center + offset * gaden::Vector3{-1, -1, -1};
                    }

                    if (x + 1 >= environment.description.dimensions.x || environment.at(x + 1, y, z) == targetState)
                    {
                        triangles.emplace_back();
                        triangles.back().normal = {1, 0, 0};
                        triangles.back().vertices[0] = center + offset * gaden::Vector3{+1, -1, -1};
                        triangles.back().vertices[1] = center + offset * gaden::Vector3{+1, +1, +1};
                        triangles.back().vertices[2] = center + offset * gaden::Vector3{+1, -1, +1};

                        triangles.emplace_back();
                        triangles.back().normal = {1, 0, 0};
                        triangles.back().vertices[0] = center + offset * gaden::Vector3{+1, +1, +1};
                        triangles.back().vertices[1] = center + offset * gaden::Vector3{+1, -1, -1};
                        triangles.back().vertices[2] = center + offset * gaden::Vector3{+1, +1, -1};
                    }
                }

                // Y
                {
                    if (y - 1 < 0 || environment.at(x, y - 1, z) == targetState)
                    {
                        triangles.emplace_back();
                        triangles.back().normal = {0, -1, 0};
                        triangles.back().vertices[0] = center + offset * gaden::Vector3{-1, -1, -1};
                        triangles.back().vertices[1] = center + offset * gaden::Vector3{+1, -1, -1};
                        triangles.back().vertices[2] = center + offset * gaden::Vector3{+1, -1, +1};

                        triangles.emplace_back();
                        triangles.back().normal = {0, -1, 0};
                        triangles.back().vertices[0] = center + offset * gaden::Vector3{+1, -1, +1};
                        triangles.back().vertices[1] = center + offset * gaden::Vector3{-1, -1, +1};
                        triangles.back().vertices[2] = center + offset * gaden::Vector3{-1, -1, -1};
                    }

                    if (y + 1 >= environment.description.dimensions.y || environment.at(x, y + 1, z) == targetState)
                    {
                        triangles.emplace_back();
                        triangles.back().normal = {0, 1, 0};
                        triangles.back().vertices[0] = center + offset * gaden::Vector3{-1, +1, -1};
                        triangles.back().vertices[1] = center + offset * gaden::Vector3{+1, +1, +1};
                        triangles.back().vertices[2] = center + offset * gaden::Vector3{+1, +1, -1};

                        triangles.emplace_back();
                        triangles.back().normal = {0, 1, 0};
                        triangles.back().vertices[0] = center + offset * gaden::Vector3{+1, +1, +1};
                        triangles.back().vertices[1] = center + offset * gaden::Vector3{-1, +1, -1};
                        triangles.back().vertices[2] = center + offset * gaden::Vector3{-1, +1, +1};
                    }
                }
            }
        }
    }

    // output
    //-----------------------

    // ASCII file
    if (argc == 5 && (strcmp(argv[4], "true") == 0 || strcmp(argv[4], "True") == 0))
    {
        std::ofstream outfile(argv[2]);
        outfile << "solid OccupancyGrid\n";
        if (!outfile.is_open())
        {
            spdlog::error("Could not open output file {}", argv[2]);
            return -1;
        }

        for (const Triangle& triangle : triangles)
        {
            outfile << fmt::format("facet normal {} {} {}\n", triangle.normal.x, triangle.normal.y, triangle.normal.z);
            outfile << "outer loop\n";

            outfile << fmt::format("vertex {} {} {}\n", triangle.vertices[0].x, triangle.vertices[0].y, triangle.vertices[0].z);
            outfile << fmt::format("vertex {} {} {}\n", triangle.vertices[1].x, triangle.vertices[1].y, triangle.vertices[1].z);
            outfile << fmt::format("vertex {} {} {}\n", triangle.vertices[2].x, triangle.vertices[2].y, triangle.vertices[2].z);

            outfile << "endloop\n";
            outfile << "endfacet\n";
        }
        outfile << "endsolid OccupancyGrid\n";
        outfile.close();
    }
    else
    {
        std::ofstream outfile(argv[2], std::ios_base::binary);
        if (!outfile.is_open())
        {
            spdlog::error("Could not open output file {}", argv[2]);
            return -1;
        }

        std::vector<char> header(80, 0x00);
        outfile.write(header.data(), 80);
        uint32_t numTriangles = triangles.size();
        outfile.write((char*)&numTriangles, sizeof(uint32_t));

        for (const Triangle& triangle : triangles)
        {
            outfile.write((char*)&triangle.normal, 3 * sizeof(float));
            outfile.write((char*)&triangle.vertices, 3 * 3 * sizeof(float));

            // this is arbitrary data you can append to a triangle, to be interpreted how you see fit (colors, maybe?)
            // we don't want to store anything, but the bytes must be there anyways
            constexpr uint16_t attributeData = 0x0000;
            outfile.write((char*)&attributeData, sizeof(uint16_t));
        }
        outfile.close();
    }

    return 0;
}