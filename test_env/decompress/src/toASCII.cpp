#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <bits/stdc++.h> 
#include <boost/format.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/copy.hpp>


int main(int argc, char *argv[]){
    if(argc!=3){
        std::cout << "Correct format is \"toASCII inputFile outputFile\"";
        return -1;
    }


    std::ifstream infile(argv[1], std::ios_base::binary);
    boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;
    inbuf.push(boost::iostreams::zlib_decompressor());
    inbuf.push(infile);
    std::stringstream decompressed;

    boost::iostreams::copy(inbuf,decompressed);

    std::ofstream outFile (argv[2], std::ios::out);

    double bufferD;
    int bufferInt;

    decompressed.read((char*) &bufferInt, sizeof(int));
    decompressed.read((char*) &bufferD, sizeof(double));
    outFile<<"env_min "<<bufferD;
    decompressed.read((char*) &bufferD, sizeof(double));
    outFile<<" "<<bufferD;
    decompressed.read((char*) &bufferD, sizeof(double));
    outFile<<" "<<bufferD<<"\n";

    decompressed.read((char*) &bufferD, sizeof(double));
    outFile<<"env_max "<<bufferD;
    decompressed.read((char*) &bufferD, sizeof(double));
    outFile<<" "<<bufferD;
    decompressed.read((char*) &bufferD, sizeof(double));
    outFile<<" "<<bufferD<<"\n";

    decompressed.read((char*) &bufferInt, sizeof(int));
    outFile<<"NumCells_XYZ "<<bufferInt;
    int cells_x=bufferInt;
    decompressed.read((char*) &bufferInt, sizeof(int));
    outFile<<" "<<bufferInt;
    int cells_y=bufferInt;
    decompressed.read((char*) &bufferInt, sizeof(int));
    outFile<<" "<<bufferInt<<"\n";
    int cells_z=bufferInt;

    decompressed.read((char*) &bufferD, sizeof(double));
    outFile<<"CellSizes_XYZ "<<bufferD;
    decompressed.read((char*) &bufferD, sizeof(double));
    outFile<<" "<<bufferD;
    decompressed.read((char*) &bufferD, sizeof(double));
    outFile<<" "<<bufferD<<"\n";
    
    decompressed.read((char*) &bufferD, sizeof(double));
    outFile<<"GasSourceLocation_XYZ "<<bufferD;
    decompressed.read((char*) &bufferD, sizeof(double));
    outFile<<" "<<bufferD;
    decompressed.read((char*) &bufferD, sizeof(double));
    outFile<<" "<<bufferD<<"\n";

    decompressed.read((char*) &bufferInt, sizeof(int));
    outFile<<"GasType "<<bufferInt<<"\n";

    outFile<<"Number of moles per filament ";
    decompressed.read((char*) &bufferD, sizeof(double));
    outFile<<bufferD<<"\nMoles of all gases in cm3 ";
    decompressed.read((char*) &bufferD, sizeof(double));
    outFile<<bufferD<<"\n";

    decompressed.read((char*) &bufferInt, sizeof(int));
    outFile<<bufferInt<<"\n";

    while(decompressed.peek()!=EOF){

        decompressed.read((char*) &bufferInt, sizeof(int));
        outFile<<bufferInt<<" ";
        decompressed.read((char*) &bufferD, sizeof(double));
        outFile<<bufferD<<" ";
        decompressed.read((char*) &bufferD, sizeof(double));
        outFile<<bufferD<<" ";
        decompressed.read((char*) &bufferD, sizeof(double));
        outFile<<bufferD<<" ";
        decompressed.read((char*) &bufferD, sizeof(double));
        outFile<<bufferD<<"\n";

    }

    outFile.close();
    return 0;
}