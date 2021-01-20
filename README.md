# Schunk lwa4p moveit 

Repository that contains ROS packages needed for successful running Schunk LWA4P Powerball in Gazebo. 

# System requirements 
Following packages have been used with **ROS kinetic**, **Ubuntu 16.04** and **Gazebo9**. 
There is available [Dockerfile](https://github.com/larics/docker_files/blob/master/ros-kinetic/moveit_ros/Dockerfile) for installing all dependencies/requirements.

# TODO:
 - [ ] Add separator to moveit configuration
 - [ ] Fix model coloring
 - [ ] Write node for dummy pose sending (send pose, reach goal, hardcode at first)
 - [ ] Think of a way to decouple separator from holder after setting it up
 - [ ] Add magnetic field to Gazebo
 - [ ] Check Path Tolerance Violated
