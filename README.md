# Schunk lwa4p moveit 

Repository that contains ROS packages needed for successful running Schunk LWA4P Powerball in Gazebo. 
Available packages are:  
 * schunk_lwa4p_control --> arm_control, action servers  
 * schunk_lwa4p_description --> urdf files  
 * schunk_lwa4p_gazebo --> launch files + controllers  
 * schunk_lwa4p_moveit_config --> ROS package for MoveIt configuration  

Joint limits and some links to HW resources can be found [here](http://wiki.ros.org/schunk_description)

# How to launch simulation? 

If you want to use robot from simulation run one of the following commands:
```
roslaunch schunk_lwa4p_gazebo lwa4p_gazebo_moveit.launch
```

# Interfacing with a robot? 

## Send commands 

## Read current arm state 

## Command current arm state 

## RVIZ control 

# ROS control 

Controllers which are used are: 

## Joint state controller 

## Joint trajectory controller 

## Joint position control

In order to use real arm, it's neccessary to properly configure 
controllers: 

Controller used for trajectory planning and execution is `jointTrajectoryController`, 
controller used for decoupled joint position moving is `jointPositionController` and 
one used for coupled joint position movement is `jointGroupPositionController`.
