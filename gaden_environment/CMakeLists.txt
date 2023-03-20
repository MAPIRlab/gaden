cmake_minimum_required(VERSION 2.8.3)
project(gaden_environment)

##################
set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++11 -g -O2 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS 
    roscpp
    visualization_msgs 
    std_msgs   
    geometry_msgs
    tf
    message_generation
)

add_service_files(
  FILES
  Occupancy.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
	DEPENDS roscpp visualization_msgs tf std_msgs
)

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
)

add_executable(environment src/environment.cpp)
add_dependencies(environment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(environment
   ${catkin_LIBRARIES}
 )
