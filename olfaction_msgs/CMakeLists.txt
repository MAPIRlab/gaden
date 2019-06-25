cmake_minimum_required(VERSION 2.8.3)
project(olfaction_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        nav_msgs
        message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
   FILES
   anemometer.msg
   gas_sensor.msg
   gas_sensor_array.msg   
)

add_service_files(
   FILES
   suggestNextObservationLocation.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  nav_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES olfaction_msgs
   CATKIN_DEPENDS std_msgs message_runtime
#  DEPENDS system_lib
)
