cmake_minimum_required(VERSION 3.13)
project(gaden_common)

set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-glldb -O0 ${CMAKE_CXX_FLAGS}")

find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_imgui REQUIRED)

add_subdirectory(third_party/gaden_core)
add_subdirectory(third_party/gaden_gui)
target_compile_definitions(gaden PUBLIC GADEN_ROS=1)
ament_target_dependencies(gaden rclcpp)

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


#Library
install(
    TARGETS gaden gaden_gui
    DESTINATION lib/${PROJECT_NAME}
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