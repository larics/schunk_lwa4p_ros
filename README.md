# Schunk lwa4p moveit 

Repository that contains ROS packages needed for successful running Schunk LWA4P Powerball in Gazebo. 

# System requirements 
Following packages have been used with **ROS kinetic**, **Ubuntu 16.04** and **Gazebo9**. 
There is available [Dockerfile](https://github.com/larics/docker_files/blob/master/ros-kinetic/moveit_ros/Dockerfile) for installing all dependencies/requirements.

# TODO:
 - [x] Add separator to moveit configuration (weird separator main BUG)
 - [x] Fix model coloring
 - [x] Plan and move arm from RVIZ in Gazebo 
 - [ ] Add powerline and magnetic field
 - [ ] Add powerline and magnetic field to possible moveit collisions (no camera = no octomap) 
 - [ ] Write node for dummy pose sending (send pose, reach goal, hardcode at first)
 - [ ] Think of a way to decouple separator from holder after setting it up (prismatic joint, possible move arm freely, dynamically change robot description? not advised).
 - [x] Check Path Tolerance Violated (Changed dynamic joint params, error reduced) 
