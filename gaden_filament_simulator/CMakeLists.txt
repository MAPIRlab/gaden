cmake_minimum_required(VERSION 3.5)
project(gaden_filament_simulator)

set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${CMAKE_CXX_FLAGS}")

if(CMAKE_BUILD_TYPE MATCHES "Debug")
    add_definitions(-DGADEN_DEBUG=1)
endif()

set(CMAKE_CXX_STANDARD 17)




find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gaden_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS iostreams filesystem)


include_directories(include)

FILE(GLOB_RECURSE MYFILES_CPP "src/*.cpp")
add_executable(filament_simulator ${MYFILES_CPP})

ament_target_dependencies(filament_simulator 
  rclcpp
  gaden_common
  std_msgs
  visualization_msgs
  Boost
)

install(
  TARGETS filament_simulator
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()