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



# Ament dependencies
find_package(ament_cmake REQUIRED)
set(AMENT_DEPS
    rclcpp
    tf2
    std_msgs
    )

if(GENERATE_COPPELIA_SCENE)
    add_definitions(-DGENERATE_COPPELIA_SCENE)
    list(APPEND AMENT_DEPS coppelia_remote_api)
endif()

foreach(Dependency IN ITEMS ${AMENT_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()




# Target
include_directories(
  include
)

add_executable(preprocessing src/preprocessing.cpp)


ament_target_dependencies(
    preprocessing 
    ${AMENT_DEPS}
)

find_package(fmt)
target_link_libraries(preprocessing fmt)

# Install
install(
  TARGETS preprocessing
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()