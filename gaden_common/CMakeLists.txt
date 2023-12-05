cmake_minimum_required(VERSION 3.5)
project(gaden_common)


find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)

ament_export_libraries(
	fmt
)
ament_export_include_directories(
    include
)

install(
    DIRECTORY include/gaden_common
    DESTINATION include
)
ament_package()