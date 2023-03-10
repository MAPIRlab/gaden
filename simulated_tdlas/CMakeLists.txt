# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 3.10)
project(simulated_tdlas)

set(CMAKE_BUILD_TYPE "None") 

set(CMAKE_CXX_FLAGS "-std=c++11 -g -O3 ${CMAKE_CXX_FLAGS}")

# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
find_package(
	catkin REQUIRED COMPONENTS
	roscpp
	visualization_msgs
	std_msgs
	nav_msgs
	olfaction_msgs
	tf
	pcl_ros
	gaden_player
)
catkin_package(
	DEPENDS 
	roscpp
	visualization_msgs
	std_msgs
	olfaction_msgs
	nav_msgs
	tf
	gaden_player
)
find_package(OpenCV)


include_directories(
	include 
	third_party/DDA/include
	third_party/DDA/third_party/glm
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)
set(SOURCES_FILES 
	src/simulated_tdlas.cpp
)

add_subdirectory(third_party/DDA/third_party/glm)

add_executable(simulated_tdlas  ${SOURCES_FILES})
add_dependencies(simulated_tdlas ${olfaction_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(simulated_tdlas
   	${catkin_LIBRARIES}
	${OpenCV_LIBS}
	glm
)
