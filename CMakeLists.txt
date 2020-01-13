cmake_minimum_required(VERSION 2.8.3)
project(odom2mavros)
add_compile_options(-std=c++11)


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
find_package (Eigen3 REQUIRED)
find_package (yaml-cpp REQUIRED)
find_package (Sophus REQUIRED )


catkin_package(
    CATKIN_DEPENDS roscpp std_msgs
    )

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    )


add_executable(node_odom2mavros src/odom2mavros_node.cpp)

target_link_libraries(node_odom2mavros
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    ${Sophus_LIBRARIES}
    )
