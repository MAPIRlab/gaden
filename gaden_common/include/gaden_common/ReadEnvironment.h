#pragma once
#include <vector>
#include <string>
#include <stdint.h>
#include <fstream>
#include <sstream>
#include "Vector3.h"

namespace Gaden
{
    static int indexFrom3D(const Vector3i& index, const Vector3i& num_cells_env)
    {
        return index.x + index.y * num_cells_env.x + index.z * num_cells_env.x * num_cells_env.y;
    }

    enum CellState : uint8_t
    {
        Free = 0,
        Obstacle = 1,
        Outlet = 2
    };

    struct EnvironmentDescription
    {
        Vector3i num_cells;
        Vector3 min_coord; //[m]
        Vector3 max_coord; //[m]
        float cell_size;   //[m]

        std::vector<uint8_t> Env;

        CellState& at(int i, int j, int k)
        {
            return (CellState&)Env[indexFrom3D({i, j, k}, num_cells)];
        }

        Vector3 coordsOfCellCenter(const Vector3i& indices)
        {
            return min_coord + (static_cast<Vector3>(indices) + 0.5f) * cell_size;
        }

        Vector3 coordsOfCellOrigin(const Vector3i& indices)
        {
            return min_coord + (static_cast<Vector3>(indices)) * cell_size;
        }
    };

    enum class ReadResult
    {
        OK,
        NO_FILE,
        READING_FAILED
    };

    static ReadResult readEnvFile(const std::string& filePath, EnvironmentDescription& desc)
    {
        if (filePath == "")
            return ReadResult::NO_FILE;

        // open file
        std::ifstream infile(filePath.c_str());
        std::string line;

        // read the header
        {
            // Line 1 (min values of environment)
            std::getline(infile, line);
            size_t pos = line.find(" ");
            line.erase(0, pos + 1);
            pos = line.find(" ");
            desc.min_coord.x = atof(line.substr(0, pos).c_str());
            line.erase(0, pos + 1);
            pos = line.find(" ");
            desc.min_coord.y = atof(line.substr(0, pos).c_str());
            desc.min_coord.z = atof(line.substr(pos + 1).c_str());

            // Line 2 (max values of environment)
            std::getline(infile, line);
            pos = line.find(" ");
            line.erase(0, pos + 1);
            pos = line.find(" ");
            desc.max_coord.x = atof(line.substr(0, pos).c_str());
            line.erase(0, pos + 1);
            pos = line.find(" ");
            desc.max_coord.y = atof(line.substr(0, pos).c_str());
            desc.max_coord.z = atof(line.substr(pos + 1).c_str());

            // Line 3 (Num cells on eahc dimension)
            std::getline(infile, line);
            pos = line.find(" ");
            line.erase(0, pos + 1);
            pos = line.find(" ");
            desc.num_cells.x = atoi(line.substr(0, pos).c_str());
            line.erase(0, pos + 1);
            pos = line.find(" ");
            desc.num_cells.y = atof(line.substr(0, pos).c_str());
            desc.num_cells.z = atof(line.substr(pos + 1).c_str());

            // Line 4 cell_size (m)
            std::getline(infile, line);
            pos = line.find(" ");
            desc.cell_size = atof(line.substr(pos + 1).c_str());
        }

        desc.Env.resize(desc.num_cells.x * desc.num_cells.y * desc.num_cells.z);

        int x_idx = 0;
        int y_idx = 0;
        int z_idx = 0;

        while (std::getline(infile, line))
        {
            std::stringstream ss(line);
            if (z_idx >= desc.num_cells.z)
            {
                printf("Too many lines! z_idx=%d but num_cells_z=%d", z_idx, desc.num_cells.z);
                return ReadResult::READING_FAILED;
            }

            if (line == ";")
            {
                // New Z-layer
                z_idx++;
                x_idx = 0;
                y_idx = 0;
            }
            else
            { // New line with constant x_idx and all the y_idx values
                while (ss)
                {
                    int f;
                    ss >> std::skipws >> f;
                    if (!ss.fail())
                    {
                        desc.Env[indexFrom3D(Vector3i(x_idx, y_idx, z_idx), desc.num_cells)] = f;
                        y_idx++;
                    }
                }

                // Line has ended
                x_idx++;
                y_idx = 0;
            }
        }
        infile.close();
        return ReadResult::OK;
    }

} // namespace Gaden