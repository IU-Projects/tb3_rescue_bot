cmake_minimum_required(VERSION 2.8.3)
project(tb3_victim_mapper)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
        nav_msgs
        geometry_msgs
        visualization_msgs
        roscpp
        rospy
        )

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_executable(victim_mapper src/victim_mapper.cpp)
add_dependencies(victim_mapper nav_msgs_generate_messages_cpp)
target_link_libraries(victim_mapper ${catkin_LIBRARIES})