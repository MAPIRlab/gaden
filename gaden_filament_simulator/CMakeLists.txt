cmake_minimum_required(VERSION 3.5)
project(gaden_filament_simulator)

set(CMAKE_BUILD_TYPE "None")

set(CMAKE_CXX_FLAGS "-std=c++17 -fopenmp ${CMAKE_CXX_FLAGS}")
set(DEBUG OFF)

if(DEBUG)
  add_definitions(-DDEBUG)
  set(CMAKE_CXX_FLAGS "-g -O0 ${CMAKE_CXX_FLAGS}")
else()
  set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
endif(DEBUG)



find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams filesystem)


include_directories(include
../gaden_common/include)

FILE(GLOB_RECURSE MYFILES_CPP "src/*.cpp")
add_executable(filament_simulator ${MYFILES_CPP})

ament_target_dependencies(filament_simulator 
  rclcpp
  std_msgs
  visualization_msgs
  Boost
)

install(
  TARGETS filament_simulator
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()