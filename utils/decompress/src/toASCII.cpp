#include <fstream>
#include <sstream>
#include <iostream>
#include <bits/stdc++.h>
#include <boost/format.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/copy.hpp>
#include "../../gaden_common/include/gaden_common/ReadEnvironment.h"

struct GasData
{
    double total_moles_in_filament;
    double num_moles_all_gases_in_cm3;
    int gas_type;
    gaden::Vector3 source_position;
};

void load_logfile_version_1(std::stringstream& decompressed, gaden::Environment& environment, GasData& gasData)
{
    // coordinates were initially written as doubles, but we want to read them as floats now, so we need a buffer
    double bufferDoubles[5];
    decompressed.read((char*)&bufferDoubles, 3 * sizeof(double));
    environment.description.min_coord.x = bufferDoubles[0];
    environment.description.min_coord.y = bufferDoubles[1];
    environment.description.min_coord.z = bufferDoubles[2];

    decompressed.read((char*)&bufferDoubles, 3 * sizeof(double));
    environment.description.max_coord.x = bufferDoubles[0];
    environment.description.max_coord.y = bufferDoubles[1];
    environment.description.max_coord.z = bufferDoubles[2];

    decompressed.read((char*)&environment.description.dimensions.x, sizeof(int));
    decompressed.read((char*)&environment.description.dimensions.y, sizeof(int));
    decompressed.read((char*)&environment.description.dimensions.z, sizeof(int));

    decompressed.read((char*)&bufferDoubles, 3 * sizeof(double));
    environment.description.cell_size = bufferDoubles[0];

    decompressed.read((char*)&bufferDoubles, 3 * sizeof(double));
    gasData.source_position.x = bufferDoubles[0];
    gasData.source_position.y = bufferDoubles[1];
    gasData.source_position.z = bufferDoubles[2];

    decompressed.read((char*)&gasData.gas_type, sizeof(int));

    decompressed.read((char*)&gasData.total_moles_in_filament, sizeof(double));
    decompressed.read((char*)&gasData.num_moles_all_gases_in_cm3, sizeof(double));
}

void load_logfile_version_2(std::stringstream& decompressed, gaden::Environment& environment, GasData& gasData)
{
    decompressed.read((char*)&environment.description, sizeof(environment.description));
    decompressed.read((char*)&gasData.source_position, sizeof(gasData.source_position));

    decompressed.read((char*)&gasData.gas_type, sizeof(int));

    decompressed.read((char*)&gasData.total_moles_in_filament, sizeof(double));
    decompressed.read((char*)&gasData.num_moles_all_gases_in_cm3, sizeof(double));
}

int main(int argc, char* argv[])
{
    if (argc != 3)
    {
        std::cout << "Correct format is \"toASCII inputFile outputFile\"";
        return -1;
    }

    std::ifstream infile(argv[1], std::ios_base::binary);
    boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;
    inbuf.push(boost::iostreams::zlib_decompressor());
    inbuf.push(infile);
    std::stringstream decompressed;

    boost::iostreams::copy(inbuf, decompressed);

    std::ofstream outFile(argv[2], std::ios::out);

    gaden::Environment environment;
    GasData gasData;
    decompressed.read((char*)&environment.versionMajor, sizeof(int));
    if (environment.versionMajor == 1)
    {
        environment.versionMinor = 0;
        load_logfile_version_1(decompressed, environment, gasData);
    }
    else
    {
        decompressed.read((char*)&environment.versionMinor, sizeof(int));
        load_logfile_version_2(decompressed, environment, gasData);
    }

    outFile << "VERSION " << environment.versionMajor << "." << environment.versionMinor << "\n";

    outFile << "env_min " << environment.description.min_coord.x;
    outFile << " " << environment.description.min_coord.y;
    outFile << " " << environment.description.min_coord.z << "\n";

    outFile << "env_max " << environment.description.max_coord.x;
    outFile << " " << environment.description.max_coord.y;
    outFile << " " << environment.description.max_coord.z << "\n";

    outFile << "NumCells_XYZ " << environment.description.dimensions.x;
    outFile << " " << environment.description.dimensions.y;
    outFile << " " << environment.description.dimensions.z << "\n";

    outFile << "CellSize " << environment.description.cell_size << "\n";

    outFile << "GasSourceLocation_XYZ " << gasData.source_position.x;
    outFile << " " << gasData.source_position.y;
    outFile << " " << gasData.source_position.z << "\n";

    outFile << "GasType " << gasData.gas_type << "\n";

    outFile << "Number of moles per filament ";
    outFile << gasData.total_moles_in_filament << "\nMoles of all gases in cm3 ";
    outFile << gasData.num_moles_all_gases_in_cm3 << "\n";

    int wind_iteration;
    decompressed.read((char*)&wind_iteration, sizeof(int));

    outFile << "Wind iteration " << wind_iteration << "\n";
    outFile << "\n-------------------------\n\n\n";
    outFile << "Filament list:\n"
               "(index) (position_x) (position_y) (position_z) (sigma)"
               "\n-------------------------\n";

    while (decompressed.peek() != EOF)
    {
        int bufferInt;
        double bufferDouble;
        decompressed.read((char*)&bufferInt, sizeof(int));
        outFile << bufferInt << " ";
        decompressed.read((char*)&bufferDouble, sizeof(double));
        outFile << bufferDouble << " ";
        decompressed.read((char*)&bufferDouble, sizeof(double));
        outFile << bufferDouble << " ";
        decompressed.read((char*)&bufferDouble, sizeof(double));
        outFile << bufferDouble << " ";
        decompressed.read((char*)&bufferDouble, sizeof(double));
        outFile << bufferDouble << "\n";
    }

    outFile.close();
    return 0;
}