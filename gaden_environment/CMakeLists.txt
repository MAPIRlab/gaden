cmake_minimum_required(VERSION 2.8.3)
project(gaden_environment)
set (CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    visualization_msgs    
    tf
)
    
catkin_package(
	DEPENDS roscpp visualization_msgs tf
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS})

add_executable(environment src/environment.cpp)

target_link_libraries(environment
   ${catkin_LIBRARIES}
 )
