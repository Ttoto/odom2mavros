cmake_minimum_required(VERSION 2.8.3)
project(vo2mavros)



## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    geometry_msgs
    )


catkin_package(
    CATKIN_DEPENDS roscpp std_msgs
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    )


add_executable(node_vo2mavros src/vo2mavros_node.cpp)

target_link_libraries(node_vo2mavros
    ${catkin_LIBRARIES}
    )
