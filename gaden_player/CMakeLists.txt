cmake_minimum_required(VERSION 2.8.3)
project(gaden_player)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    std_msgs
    nav_msgs
    tf
    message_generation
    gaden_msgs
    olfaction_msgs
)


catkin_package(
    DEPENDS roscpp visualization_msgs std_msgs nav_msgs tf gaden_msgs olfaction_msgs
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
set(MYFILES_CPP src/simulation_player.cpp)

add_executable(gaden_player ${MYFILES_CPP})

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
