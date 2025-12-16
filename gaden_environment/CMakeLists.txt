cmake_minimum_required(VERSION 3.5)
project(gaden_environment)

##################
set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_RELEASE "-O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0 ${CMAKE_CXX_FLAGS}")



set(CMAKE_CXX_STANDARD 20)

# Ament dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake REQUIRED)
ament_auto_find_build_dependencies()
find_package(OpenMP REQUIRED)

include_directories(include)

ament_auto_add_executable(environment src/environment.cpp)

target_link_libraries(environment OpenMP::OpenMP_CXX)
  
install(
  TARGETS environment
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()