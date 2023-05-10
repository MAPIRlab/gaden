cmake_minimum_required(VERSION 3.5)
project(simulated_anemometer)

set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS}")

set(DEBUG OFF)

if(DEBUG)
  add_definitions(-DDEBUG)
  set(CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}")
else()
  set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
endif(DEBUG) 



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(olfaction_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(gaden_player REQUIRED)


include_directories(include)

add_executable(simulated_anemometer src/fake_anemometer.cpp)

ament_target_dependencies(simulated_anemometer
    rclcpp
    olfaction_msgs
    visualization_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    gaden_player
) 


install(
  TARGETS simulated_anemometer
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()