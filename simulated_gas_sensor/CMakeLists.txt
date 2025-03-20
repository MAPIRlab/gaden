cmake_minimum_required(VERSION 3.5)
project(simulated_gas_sensor)

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
find_package(gaden_player REQUIRED)
find_package(gaden_msgs REQUIRED)
find_package(gaden_common REQUIRED)


include_directories(include)

FILE(GLOB_RECURSE MYFILES_CPP "src/*.cpp")

add_executable(simulated_gas_sensor  ${MYFILES_CPP})


ament_target_dependencies(simulated_gas_sensor
    rclcpp
    olfaction_msgs
    visualization_msgs
    geometry_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    gaden_player
    gaden_common
    gaden_msgs
) 


install(
  TARGETS simulated_gas_sensor
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()