cmake_minimum_required(VERSION 3.5)
project(gaden_filament_simulator)

set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${CMAKE_CXX_FLAGS}")

set(CMAKE_CXX_STANDARD 20)

# Ament dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
ament_auto_find_build_dependencies()


# Main Target
ament_auto_add_executable(filament_simulator src/filament_simulator.cpp)

install(
  TARGETS filament_simulator
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()

