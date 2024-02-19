cmake_minimum_required(VERSION 3.10)
project(OccupancyGridToSTL)

set(CMAKE_CXX_FLAGS "-w -std=c++17 -O3 -g ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake)
find_package(spdlog)
find_package(gaden_common)

include_directories(
    include
)


add_executable(OccupancyGridToSTL src/OccupancyGridToSTL.cpp)
ament_target_dependencies(OccupancyGridToSTL gaden_common)
target_link_libraries(OccupancyGridToSTL spdlog::spdlog)
