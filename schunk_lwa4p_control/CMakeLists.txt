cmake_minimum_required(VERSION 3.0.2)
project(schunk_lwa4p_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    actionlib_msgs
    eigen_conversions
    geometry_msgs
    message_generation
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_move_group
    moveit_msgs
    moveit_servo
    roscpp
    separator_end_effector
    std_msgs
    tf
    tf_conversions
    trajectory_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)

#add_message_files()
add_service_files(
        DIRECTORY srv
        FILES CartesianPath.srv
)

add_action_files(
        DIRECTORY action
        FILES GoToPose.action SetupDistancer.action ServoTrackPose.action CollectMeasurements.action
)

generate_messages(
        DEPENDENCIES actionlib_msgs std_msgs geometry_msgs

)

catkin_package(
  INCLUDE_DIRS include action
  LIBRARIES
    ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
    actionlib_msgs
    eigen_conversions
    geometry_msgs
    message_runtime
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_move_group
    moveit_msgs
    moveit_servo
    roscpp
    separator_end_effector      # You don't need this if you won't use separator end effector, also
    std_msgs
    tf
    tf_conversions
    trajectory_msgs
  DEPENDS
    Eigen3
    Boost
    message_runtime
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

# GoToPose
add_executable(
        gotopose_server
        src/gotopose_server.cpp
        )

target_link_libraries(
        gotopose_server
        ${catkin_LIBRARIES}
)

add_dependencies(
        gotopose_server
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# SetupDistancer
add_executable(
    setupdistancer_server
    src/setupdistancer_server.cpp
)

target_link_libraries(
        setupdistancer_server
        ${catkin_LIBRARIES}
)

add_dependencies(
        setupdistancer_server
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# ServoTrackPose
add_executable(
        servotrackpose_server
        src/servotrackpose_server.cpp
)

target_link_libraries(
        servotrackpose_server
        ${catkin_LIBRARIES}
)

add_dependencies(
        servotrackpose_server
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

# CollectMeasurements
add_executable(
        collectmeasurements_server
        src/collectmeasurements_server.cpp
)

target_link_libraries(
        collectmeasurements_server
        ${catkin_LIBRARIES}
)

add_dependencies(
        collectmeasurements_server
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
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

target_link_libraries(
        ${PROJECT_NAME}_node
        ${PROJECT_NAME}_lib
        ${Boost_LIBRARIES}
        ${EIGEN3_LIBRARIES}
        ${catkin_LIBRARIES}
)




