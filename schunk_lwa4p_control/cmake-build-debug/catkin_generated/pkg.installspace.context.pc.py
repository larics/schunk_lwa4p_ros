# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include".split(';') if "${prefix}/include" != "" else []
PROJECT_CATKIN_DEPENDS = "roscpp;std_msgs;geometry_msgs;moveit_msgs;moveit_core;moveit_visual_tools;moveit_ros_planning;moveit_ros_planning_interface;moveit_ros_move_group;tf;tf_conversions;trajectory_msgs;eigen_conversions".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lschunk_lwa4p_control_lib".split(';') if "-lschunk_lwa4p_control_lib" != "" else []
PROJECT_NAME = "schunk_lwa4p_control"
PROJECT_SPACE_DIR = "/usr/local"
PROJECT_VERSION = "0.0.0"
