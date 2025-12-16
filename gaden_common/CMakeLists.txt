cmake_minimum_required(VERSION 3.13)
project(gaden_common)

find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Compile the backend (gaden_core) library
#--------------------------------------------------------
add_subdirectory(third_party/gaden_core)
target_compile_definitions(gaden PUBLIC GADEN_ROS=1) # to make sure rclcpp is used for logging
ament_target_dependencies(gaden rclcpp std_msgs visualization_msgs)

# Optional! You can comment out this line to disable compilation of the gaden_gui executable
#--------------------------------------------------------
include(cmake/gui.cmake)


# set up the libraries / includes for downstream packages
#--------------------------------------------------------
ament_export_libraries(
	fmt
    gaden
)
ament_export_include_directories(
    include
    third_party/DDA/include
    third_party/DDA/third_party/glm
    third_party/libbsc
)

get_target_property(GADEN_DEFS gaden COMPILE_DEFINITIONS)
ament_export_definitions(GADEN_ROS=1)
if(GADEN_DEFS MATCHES "TRACY_ENABLE")
    ament_export_definitions(TRACY_ENABLE)
endif()
if(GADEN_DEFS MATCHES "GPU_ACCELERATION")
    ament_export_definitions(GPU_ACCELERATION=1)
endif()

# Install libraries / include dirs
#--------------------------------------------------------
#Library
install(
    TARGETS gaden 
    DESTINATION lib
)

# include directories
install(
    DIRECTORY include/gaden_common
    DESTINATION include
)

install(
    DIRECTORY third_party/gaden_core/include/gaden
    DESTINATION include
)

install(
    DIRECTORY third_party/gaden_core/third_party/DDA
    DESTINATION third_party
)

install(
    DIRECTORY third_party/gaden_core/third_party/libbsc
    DESTINATION third_party
)

# python module for launch files
install(
    DIRECTORY gaden_internal_py
    DESTINATION share/${PROJECT_NAME}
)

ament_package()