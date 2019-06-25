cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_FLAGS "-lboost_iostreams -std=c++11 ${CMAKE_CXX_FLAGS}")
project(gaden_player)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    std_msgs
    nav_msgs
    tf
    message_generation
)
find_package(Boost REQUIRED COMPONENTS iostreams)
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
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(gaden_player  ${MYFILES_CPP})
add_dependencies(gaden_player ${olfaction_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
