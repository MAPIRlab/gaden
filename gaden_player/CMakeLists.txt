cmake_minimum_required(VERSION 3.5)
project(gaden_player)

set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${CMAKE_CXX_FLAGS}")


set(CMAKE_CXX_STANDARD 20)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(player src/simulation_player.cpp)


install(
  TARGETS player
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
