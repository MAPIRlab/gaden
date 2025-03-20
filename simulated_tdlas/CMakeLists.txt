# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 3.10)
project(simulated_tdlas)

set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${CMAKE_CXX_FLAGS}")


set(CMAKE_CXX_STANDARD 17)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(olfaction_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(gaden_common REQUIRED)
find_package(gaden_player REQUIRED)
find_package(gaden_msgs REQUIRED)
find_package(gaden_environment REQUIRED)


include_directories(
	include 
)

set(SOURCES_FILES 
	src/simulated_tdlas.cpp
)

add_executable(simulated_tdlas  ${SOURCES_FILES})


ament_target_dependencies(simulated_tdlas
    rclcpp
    olfaction_msgs
    visualization_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    gaden_common
    gaden_player
    gaden_environment
    gaden_msgs
) 


install(
  TARGETS simulated_tdlas
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()