
cmake_minimum_required(VERSION 3.8)
project(gaden_msgs)


find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
    msg/GasInCell.msg
    srv/GasPosition.srv
    srv/WindPosition.srv
    srv/Occupancy.srv
    DEPENDENCIES std_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()

