#pragma once
#include "GadenVersion.h"
#include "Vector3.h"
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdint.h>
#include <string>
#include <vector>

namespace gaden
{
    inline size_t indexFrom3D(const Vector3i& index, const Vector3i& num_cells_env)
    {
        return index.x + index.y * num_cells_env.x + index.z * num_cells_env.x * num_cells_env.y;
    }

    enum class CellState : uint8_t
    {
        Free = 0,       // cell is empty, gas can be here
        Obstacle = 1,   // cell is occupied by an obstacle, no filaments can go through it
        Outlet = 2,     // if a filament enters this cell it is removed from the simulation
        OutOfBounds = 3 // invalid cell, position is out of the map bounds
    };

    struct Environment
    {
        int versionMajor = gaden::version_major,
            versionMinor = gaden::version_minor; // version of gaden used to generate a log file. Used to figure out how to parse the binary format
        struct Description
        {
            Vector3i dimensions;
            Vector3 min_coord; //[m]
            Vector3 max_coord; //[m]
            float cell_size;   //[m]
        };
        Description description;

        std::vector<uint8_t> Env;

        size_t numCells() const
        {
            return description.dimensions.x * description.dimensions.y * description.dimensions.z;
        }

        CellState& at(int i, int j, int k)
        {
            return (CellState&)Env[indexFrom3D({i, j, k}, description.dimensions)];
        }

        Vector3 coordsOfCellCenter(const Vector3i& indices) const
        {
            return description.min_coord + (static_cast<Vector3>(indices) + 0.5f) * description.cell_size;
        }

        Vector3 coordsOfCellOrigin(const Vector3i& indices) const
        {
            return description.min_coord + (static_cast<Vector3>(indices)) * description.cell_size;
        }
    };

    enum class ReadResult
    {
        OK,
        NO_FILE,
        READING_FAILED
    };

    inline ReadResult readEnvFile(const std::string& filePath, Environment& environment)
    {
        if (!std::filesystem::exists(filePath))
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
            environment.description.min_coord.x = atof(line.substr(0, pos).c_str());
            line.erase(0, pos + 1);
            pos = line.find(" ");
            environment.description.min_coord.y = atof(line.substr(0, pos).c_str());
            environment.description.min_coord.z = atof(line.substr(pos + 1).c_str());

            // Line 2 (max values of environment)
            std::getline(infile, line);
            pos = line.find(" ");
            line.erase(0, pos + 1);
            pos = line.find(" ");
            environment.description.max_coord.x = atof(line.substr(0, pos).c_str());
            line.erase(0, pos + 1);
            pos = line.find(" ");
            environment.description.max_coord.y = atof(line.substr(0, pos).c_str());
            environment.description.max_coord.z = atof(line.substr(pos + 1).c_str());

            // Line 3 (Num cells on eahc dimension)
            std::getline(infile, line);
            pos = line.find(" ");
            line.erase(0, pos + 1);
            pos = line.find(" ");
            environment.description.dimensions.x = atoi(line.substr(0, pos).c_str());
            line.erase(0, pos + 1);
            pos = line.find(" ");
            environment.description.dimensions.y = atof(line.substr(0, pos).c_str());
            environment.description.dimensions.z = atof(line.substr(pos + 1).c_str());

            // Line 4 cell_size (m)
            std::getline(infile, line);
            pos = line.find(" ");
            environment.description.cell_size = atof(line.substr(pos + 1).c_str());
        }

        environment.Env.resize(environment.description.dimensions.x * environment.description.dimensions.y * environment.description.dimensions.z);

        int x_idx = 0;
        int y_idx = 0;
        int z_idx = 0;

        while (std::getline(infile, line))
        {
            std::stringstream ss(line);
            if (z_idx >= environment.description.dimensions.z)
            {
                printf("Too many lines! z_idx=%d but num_cells_z=%d", z_idx, environment.description.dimensions.z);
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
                        environment.Env[indexFrom3D(Vector3i(x_idx, y_idx, z_idx), environment.description.dimensions)] = f;
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

} // namespace gaden