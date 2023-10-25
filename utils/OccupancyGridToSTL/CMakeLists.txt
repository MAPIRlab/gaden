cmake_minimum_required(VERSION 3.10)
project(OccupancyGridToSTL)

set(CMAKE_CXX_FLAGS "-w -std=c++17 -O3 -g ${CMAKE_CXX_FLAGS}")

find_package(spdlog)
find_package(fmt)

include_directories(
    include
    ../../gaden_common/include
)

add_executable(OccupancyGridToSTL src/OccupancyGridToSTL.cpp)
target_link_libraries(OccupancyGridToSTL spdlog::spdlog fmt)
