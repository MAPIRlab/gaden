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

install(
    DIRECTORY third_party/DDA/include/DDA
    DESTINATION include/gaden_common/third_party
)

install(
    DIRECTORY third_party/DDA/third_party/glm/glm
    DESTINATION include/gaden_common/third_party/glm
)

install(
    DIRECTORY gaden_internal_py
    DESTINATION share/${PROJECT_NAME}
)

ament_package()