# Schunk lwa4p moveit 

Repository that contains ROS packages needed for successful running Schunk LWA4P Powerball in Gazebo. 
Available packages are:  
 * schunk_lwa4p_control --> arm_control, action servers  
 * schunk_lwa4p_description --> urdf files  
 * schunk_lwa4p_gazebo --> launch files + controllers  
 * schunk_lwa4p_moveit_config --> ROS package for MoveIt configuration  

# How to launch simulation? 

If you want to use robot from simulation run one of the following commands:
```
roslaunch schunk_lwa4p_gazebo lwa4p_gazebo_moveit.launch
```

Lanuch simulation with robotic arm and powerline model:
```
roslaunch schunk_lwa4p_gazebo lwa4p_powerline_gazebo_moveit.launch
```

Launch real robot: 
```
roslaunch schunk_lwa4p_gazebo lwa4p_real_robot_moveit.launch
```

# System requirements 
Following packages have been used with ROS melodic, Gazebo 11 on Ubuntu 18.04. 
There is available [Dockerfile](https://github.com/larics/docker_files/blob/master/ros-melodic/moveit_ros/Dockerfile) for installing all dependencies/requirements.

# Dependencies 

If using `power line model` it's neccessary to download, source and build Gazebo plugin from this [link](https://github.com/goranvasilj/power_line_simulation/blob/master/src/power_line_simulation.cpp)
in source folder of catkin workspace. **This dependency is not written in CMakeLists.txt.** After cloning repo in source,
build it and add it to as follows: 
```
export GAZEBO_PLUGIN_PATH:=<path_to_catkin_ws>/catkin_ws/devel/lib/powerline_simulation/:$GAZEBO_PLUGIN_PATH
```

Rest of the GAZEBO plugins from workspace is located in:
```
<path_to_catkin_ws>/catkin_ws/devel/lib
```
Currently is not used! 

# TODO:
 - [x] Add separator to moveit configuration (weird separator main BUG)
 - [x] Fix model coloring
 - [x] Plan and move arm from RVIZ in Gazebo
 - [x] Decouple models for separator tool and separator distancer to enable setting it up  
 - [x] Reconfigure separator holder collisions 
 - [x] Init separator distancer in separator holder
 - [x] Add powerline to robot model for collisions
 - [x] Write node for dummy pose sending (send pose, reach goal, hardcode at first)
 - [x] Create action server for moveToLine, rotateTool, setUpDistancer -> moveGroup already has those
 - [x] Think of a way to decouple separator from holder after setting it up (prismatic joint, possible move arm freely, dynamically change robot description? not advised).
 - [x] Check Path Tolerance Violated (Changed dynamic joint params, error reduced) 
 - [x] Create Gazebo world with powerlines (robot model) 
 - [x] Integrate descartes cartesian planner
 - [x] use TracIK for IK
 - [x] Build and use OpenRAVE
 - [x] Reconfigured schunk_robots to match new joint names
 - [x] Added MoveToPose action server  
 - [ ] Create configuration for FastIK (analytical inverse kinematics for Descartes) 
 - [ ] Create node for continuous replanning 
 
# Possible improvements 

 - [x] Add powerline as Git submodule into schunk_ros_package, and use it if neccessary (not neccessary) 
 - [ ] Think of algorithm for removing separator distancer (information about powerline location is not enough)  
