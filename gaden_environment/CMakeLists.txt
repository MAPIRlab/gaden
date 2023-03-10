cmake_minimum_required(VERSION 2.8.3)
project(gaden_environment)
set (CMAKE_CXX_STANDARD 11)

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
