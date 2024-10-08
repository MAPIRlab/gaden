cmake_minimum_required(VERSION 3.5)
project(gaden_environment)

##################
set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${CMAKE_CXX_FLAGS}")


set(CMAKE_CXX_STANDARD 17)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gaden_msgs REQUIRED)
find_package(gaden_common REQUIRED)

include_directories(include)

add_executable(environment src/environment.cpp)



ament_target_dependencies(environment
    rclcpp
    std_msgs
    visualization_msgs
    geometry_msgs
    gaden_common
    gaden_msgs
) 

  
install(
  TARGETS environment
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()