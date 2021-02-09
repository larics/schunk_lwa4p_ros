cmake_minimum_required(VERSION 3.0.2)
project(schunk_lwa4p_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp 
    std_msgs
    moveit_msgs
    geometry_msgs
)
## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS
    actionlib
    roscpp
    tf
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_move_group
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}_lib
  CATKIN_DEPENDS 
    roscpp 
    std_msgs
    geometry_msgs
    moveit_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_move_group
    tf
  DEPENDS
    Eigen3
)

include_directories(
 include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
)

###########
## Build ##
###########

set(PROJECT_LIB_FILES 
    src/control_arm.cpp 
)

## Declare a C++ library
add_library(
    ${PROJECT_NAME}_lib
    ${PROJECT_LIB_FILES}
)

add_executable(
    ${PROJECT_NAME}_node
    src/control_arm_node.cpp 
)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}_lib
    ${Boost_LIBRARIES}
    ${EIGEN3_LIBRARIES}
    ${catkin_LIBRARIES}
)



