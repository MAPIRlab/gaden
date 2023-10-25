cmake_minimum_required(VERSION 3.10)
project(decompress)

set(CMAKE_CXX_FLAGS "-w -std=c++17 -O3 -g ${CMAKE_CXX_FLAGS}")


find_package(Boost REQUIRED COMPONENTS iostreams)

include_directories(
    include
    ../../gaden_common/include
)

add_executable(decompress src/comp.cpp)
add_executable(toASCII src/toASCII.cpp)

target_link_libraries(decompress Boost::iostreams)
target_link_libraries(toASCII Boost::iostreams)
