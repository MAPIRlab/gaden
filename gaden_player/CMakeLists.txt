# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(gaden_player)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    visualization_msgs
    std_msgs
    nav_msgs
    msgs_and_srvs
    tf)


FILE(GLOB_RECURSE MYFILES_CPP "src/*.cpp")

catkin_package(
    DEPENDS roscpp visualization_msgs std_msgs nav_msgs msgs_and_srvs tf
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(gaden_player  ${MYFILES_CPP})
add_dependencies(gaden_player msgs_and_srvs_generate_messages_cpp)

target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${catkin_LIBRARIES})
