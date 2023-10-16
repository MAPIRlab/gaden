cmake_minimum_required(VERSION 3.5)
project(gaden_preprocessing)
set(CMAKE_BUILD_TYPE "None")
set(CMAKE_CXX_FLAGS "-std=c++17 -fopenmp ${CMAKE_CXX_FLAGS}")


# OPTIONS!
#==============================================
set(GENERATE_COPPELIA_SCENE ON)
set(DEBUG OFF)
#==============================================




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
find_package(fmt)




include_directories(
  include
  ../gaden_common/include
)

# Main Target
add_executable(preprocessing src/preprocessing.cpp)
ament_target_dependencies(preprocessing	${AMENT_DEPS})
target_link_libraries(preprocessing fmt)

# Coppelia utils
add_executable(configureCoppeliaSim src/configureCoppeliaSim.cpp)
ament_target_dependencies(configureCoppeliaSim ${AMENT_DEPS})
target_link_libraries(configureCoppeliaSim fmt)



# Install
install(
  TARGETS preprocessing configureCoppeliaSim
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()