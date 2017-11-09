cmake_minimum_required(VERSION 2.8.3)
project(gaden_msgs)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    std_msgs
    nav_msgs
    tf
    message_generation
)

add_service_files(
  FILES
  GasPosition.srv
  WindPosition.srv
  SimulationIterationRequest.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    DEPENDS roscpp visualization_msgs std_msgs nav_msgs tf
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

