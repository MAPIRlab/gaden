cmake_minimum_required(VERSION 2.8.3)
project(preprocessing)

set(CMAKE_CXX_FLAGS "-std=c++11 -g ${CMAKE_CXX_FLAGS}")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)
find_package(Eigen3 REQUIRED)
catkin_package(
)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(preprocessing src/preprocessing.cpp)
target_link_libraries(preprocessing
   ${catkin_LIBRARIES}
	 Eigen3::Eigen
)
