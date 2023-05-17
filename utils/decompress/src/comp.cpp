#include <fstream>
#include <iostream>
#include <vector>
#include <bits/stdc++.h> 
#include <boost/format.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/copy.hpp>

int main(int argc, char *argv[]){

    using namespace std;

    if(argc!=3){
        cout << "Correct format is \"decompress inputFile outputFile\"";
    }else{
        ifstream infile(argv[1], ios_base::binary);
        boost::iostreams::filtering_streambuf<boost::iostreams::input> inbuf;
        inbuf.push(boost::iostreams::zlib_decompressor());
        inbuf.push(infile);
        ofstream out(argv[2]);
        boost::iostreams::copy(inbuf,out);
    }
    
}
