cmake_minimum_required(VERSION 3.5)
project(gaden_preprocessing)

set(CMAKE_BUILD_TYPE "None")

set(CMAKE_CXX_FLAGS "-std=c++17 -fopenmp ${CMAKE_CXX_FLAGS}")
set(DEBUG OFF)

if(DEBUG)
  add_definitions(-DDEBUG)
  set(CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}")
else()
  set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
endif(DEBUG)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(
  include
)

add_executable(preprocessing src/preprocessing.cpp)


ament_target_dependencies(preprocessing 
  rclcpp
  tf2
  std_msgs
)


install(
  TARGETS preprocessing
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()