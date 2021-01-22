# Schunk lwa4p moveit 

Repository that contains ROS packages needed for successful running Schunk LWA4P Powerball in Gazebo. 

# System requirements 
Following packages have been used with **ROS kinetic**, **Ubuntu 16.04** and **Gazebo9**. 
There is available [Dockerfile](https://github.com/larics/docker_files/blob/master/ros-kinetic/moveit_ros/Dockerfile) for installing all dependencies/requirements.

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
 - [ ] Reconfigure separator holder collisions 
 - [ ] Init separator distancer in separator holder
 - [/] Add powerline and magnetic field --> Simulation postponed
 - [ ] Add powerline and magnetic field to possible moveit collisions (no camera = no octomap) 
 - [ ] Write node for dummy pose sending (send pose, reach goal, hardcode at first)
 - [ ] Create action server for moveToLine, rotateTool, setUpDistancer
 - [ ] Think of a way to decouple separator from holder after setting it up (prismatic joint, possible move arm freely, dynamically change robot description? not advised).
 - [x] Check Path Tolerance Violated (Changed dynamic joint params, error reduced) 
 - [ ] Create node for continuous replanning 

# Possible improvements 

 - [ ] Add powerline as Git submodule into schunk_ros_package, and use it if neccessary
 - [ ] Think of algorithm for removing separator distancer (information about powerline location is not enough)  
