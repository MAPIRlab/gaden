cmake_minimum_required(VERSION 3.5)
project(gaden_environment)

##################
set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++17 -g -O2 ${CMAKE_CXX_FLAGS}")


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gaden_common REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/Occupancy.srv
  DEPENDENCIES geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(
    include
)

add_executable(environment src/environment.cpp)



ament_target_dependencies(environment
    rclcpp
    std_msgs
    visualization_msgs
    geometry_msgs
    gaden_common
) 

# dependency for self-defined messages

if("${ROS_DISTRO}" STREQUAL "foxy")
  rosidl_target_interfaces(environment
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(environment "${cpp_typesupport_target}") 
endif()

  
install(
  TARGETS environment
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()