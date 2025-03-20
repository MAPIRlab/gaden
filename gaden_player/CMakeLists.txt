cmake_minimum_required(VERSION 3.5)
project(gaden_player)

set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${CMAKE_CXX_FLAGS}")


set(CMAKE_CXX_STANDARD 17)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gaden_common REQUIRED)
find_package(gaden_msgs REQUIRED)

find_package(Boost REQUIRED COMPONENTS iostreams)

include_directories(include)

FILE(GLOB_RECURSE MYFILES_CPP "src/*.cpp")
add_executable(player  ${MYFILES_CPP})


ament_target_dependencies(player
    rclcpp
    std_msgs
    visualization_msgs
    Boost
    gaden_common
    gaden_msgs
) 
target_link_libraries(player fmt)


install(
  TARGETS player
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
