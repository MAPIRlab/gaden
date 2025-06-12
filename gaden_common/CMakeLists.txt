cmake_minimum_required(VERSION 3.13)
project(gaden_common)

find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)

# Compile the backend (gaden_core) library
#--------------------------------------------------------
add_subdirectory(third_party/gaden_core)
target_compile_definitions(gaden PUBLIC GADEN_ROS=1) # to make sure rclcpp is used for logging
ament_target_dependencies(gaden rclcpp)

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
)

ament_export_definitions(GADEN_ROS=1)


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

# python module for launch files
install(
    DIRECTORY gaden_internal_py
    DESTINATION share/${PROJECT_NAME}
)

ament_package()