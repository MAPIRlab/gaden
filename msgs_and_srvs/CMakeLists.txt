cmake_minimum_required(VERSION 2.8.3)
project(msgs_and_srvs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  message_generation
)

#add_message_files(
#  FILES
#  myMSG.msg
#)


add_service_files(
  FILES
  GasPosition.srv
  WindPosition.srv
)


include_directories(${Boost_INCLUDE_DIRS})
generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package()
