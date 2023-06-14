cmake_minimum_required(VERSION 3.5)
project(gaden_preprocessing)
set(CMAKE_BUILD_TYPE "None")
set(CMAKE_CXX_FLAGS "-std=c++17 -fopenmp ${CMAKE_CXX_FLAGS}")


set(GENERATE_COPPELIA_SCENE OFF)
set(DEBUG OFF)




if(DEBUG)
  add_definitions(-DDEBUG)
  set(CMAKE_CXX_FLAGS "-g -O0 ${CMAKE_CXX_FLAGS}")
else()
  set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
endif()


if(GENERATE_COPPELIA_SCENE)
    add_definitions(-DGENERATE_COPPELIA_SCENE)
    find_package(coppelia_remote_api REQUIRED) 
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
)

add_executable(preprocessing src/preprocessing.cpp)





set(AMENT_DEPS
    rclcpp
    tf2
    std_msgs
)
if(GENERATE_COPPELIA_SCENE)
    list(APPEND AMENT_DEPS coppelia_remote_api)
    target_link_libraries(preprocessing zmq)
endif()


ament_target_dependencies(
    preprocessing 
    ${AMENT_DEPS}
)


install(
  TARGETS preprocessing
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()