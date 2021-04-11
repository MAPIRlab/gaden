cmake_minimum_required(VERSION 2.8.3)
project(simulated_anemometer)

find_package(
    catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    std_msgs
    nav_msgs
    olfaction_msgs
    tf
    gaden_player
)

FILE(GLOB_RECURSE MYFILES_CPP "src/*.cpp")

catkin_package(
    DEPENDS 
    roscpp 
    visualization_msgs 
    std_msgs 
    nav_msgs
    olfaction_msgs
    tf
    gaden_player
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(simulated_anemometer  ${MYFILES_CPP})
add_dependencies(simulated_anemometer ${olfaction_msgs_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
