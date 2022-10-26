
cmake_minimum_required(VERSION 2.8.3)
project(gaden_filament_simulator CXX)

set(CMAKE_CXX_FLAGS "-lboost_iostreams -std=c++11 -fopenmp ${CMAKE_CXX_FLAGS}")

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
	visualization_msgs
	std_msgs
	nav_msgs
	pcl_ros
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams)
catkin_package(
    DEPENDS roscpp visualization_msgs std_msgs nav_msgs Boost
)

include_directories(include)

include_directories(${catkin_INCLUDE_DIRS} $(PCL_INCLUDE_DIRS) ${OpenCV_INCLUDE_DIRS})
MESSAGE("PCL ARE: ${PCL_INCLUDE_DIRS}")

FILE(GLOB_RECURSE MYFILES_CPP "src/*.cpp")
add_executable(filament_simulator ${MYFILES_CPP})

target_link_libraries(filament_simulator
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
 )
