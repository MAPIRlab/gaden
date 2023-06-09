cmake_minimum_required(VERSION 2.8.3)
project(gaden_player)

set(CMAKE_CXX_FLAGS "-lboost_iostreams -std=c++11 ${CMAKE_CXX_FLAGS}")

set(DEBUG OFF)

if(DEBUG)
  add_definitions(-DDEBUG)
  set(CMAKE_CXX_FLAGS "-g ${CMAKE_CXX_FLAGS}")
else()
  set(CMAKE_CXX_FLAGS "-O3 ${CMAKE_CXX_FLAGS}")
endif(DEBUG) 

find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    std_msgs
    nav_msgs
    tf
    message_generation
    OpenCV
)
find_package(Boost REQUIRED COMPONENTS iostreams)

add_message_files(
  FILES
  GasInCell.msg
)

add_service_files(
  FILES
  GasPosition.srv
  WindPosition.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)


FILE(GLOB_RECURSE MYFILES_CPP "src/*.cpp")
catkin_package(
    DEPENDS roscpp visualization_msgs std_msgs nav_msgs tf Boost
    CATKIN_DEPENDS message_runtime std_msgs
)

include_directories(
    include 
    ../gaden_common/include
    ${Boost_INCLUDE_DIRS} 
    ${catkin_INCLUDE_DIRS} 
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(gaden_player  ${MYFILES_CPP})
add_dependencies(gaden_player ${${PROJECT_NAME}_EXPORTED_TARGETS} ${olfaction_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
