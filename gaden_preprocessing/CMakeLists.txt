cmake_minimum_required(VERSION 2.8.3)
project(gaden_preprocessing)

set(CMAKE_CXX_FLAGS "-std=c++11 -fopenmp ${CMAKE_CXX_FLAGS}")

option ( DEBUG "" OFF)
set(DEBUG OFF)

if(DEBUG)
  add_definitions(-DDEBUG)
  set(CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}")
else()
  set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
endif(DEBUG)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
    DEPENDS roscpp std_msgs EIGEN3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_executable(preprocessing src/preprocessing.cpp)
add_dependencies(preprocessing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(preprocessing
   ${catkin_LIBRARIES}
)
