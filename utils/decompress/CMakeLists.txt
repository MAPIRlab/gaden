cmake_minimum_required(VERSION 3.10)
project(gaden_decompress)

set(CMAKE_CXX_FLAGS "-w -std=c++17 -O3 -g ${CMAKE_CXX_FLAGS}")


find_package(ament_cmake)
find_package(Boost REQUIRED COMPONENTS iostreams)
find_package(gaden_common)

include_directories(
    include
)

add_executable(decompress src/comp.cpp)
add_executable(toASCII src/toASCII.cpp)

ament_target_dependencies(decompress gaden_common)
ament_target_dependencies(toASCII gaden_common)
target_link_libraries(decompress Boost::iostreams)
target_link_libraries(toASCII Boost::iostreams)

ament_package()