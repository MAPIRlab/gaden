cmake_minimum_required(VERSION 3.5)
project(gaden_preprocessing)
set(CMAKE_VERBOSE_MAKEFILE ON) #Outputs the compiler command to the build log file


# OPTIONS!
#==============================================
set(GENERATE_COPPELIA_SCENE OFF)
#==============================================




set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${CMAKE_CXX_FLAGS}")


set(CMAKE_CXX_STANDARD 17)




# Ament dependencies
find_package(ament_cmake REQUIRED)
set(AMENT_DEPS
    rclcpp
    tf2
	tf2_geometry_msgs
    std_msgs
    gaden_common
    yaml_cpp_vendor
    )

if(GENERATE_COPPELIA_SCENE)
    add_definitions(-DGENERATE_COPPELIA_SCENE)
    list(APPEND AMENT_DEPS coppelia_remote_api)
endif()

foreach(Dependency IN ITEMS ${AMENT_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(OpenMP)



include_directories(
  src
)
    
    # Main Target
add_executable(preprocessing src/preprocessing.cpp)
ament_target_dependencies(preprocessing	${AMENT_DEPS})
target_link_libraries(preprocessing OpenMP::OpenMP_CXX)
 
# Coppelia utils
add_executable(configureCoppeliaSim src/configureCoppeliaSim.cpp)
ament_target_dependencies(configureCoppeliaSim ${AMENT_DEPS})



# Install
install(
  TARGETS preprocessing configureCoppeliaSim
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()