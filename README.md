# Fundamentals of robotics with Schunk LWA4p and WSG50 gripper

Repository contains ROS packages needed for successful running Schunk LWA4P Powerball with WSG50 gripper in Gazebo. 
Available packages are:  
 * `schunk_lwa4p_control` --> arm_control, action servers 
 * `schunk_lwa4p_description` --> xacro/urdf files (robot model and meshes)  
 * `schunk_lwa4p_gazebo` --> launch files 
 * `schunk_lwa4p_moveit_config` --> ROS package for MoveIt configuration  

Joint limits and some links to HW resources can be found [here](http://wiki.ros.org/schunk_description). 

To run the simulation, you also need https://github.com/nalt/wsg50-ros-pkg to simulate the gripper correctly.

## Useful resources: 

* [Brief ROS introduction](https://fzoric8.github.io/2021/05/27/ROS-in-10-minutes.html) 
* [Basic MoveIt! concepts](https://moveit.ros.org/documentation/concepts/) 
* [Basic ROS control concepts](http://wiki.ros.org/ros_control) 

# How to launch simulation? 

If you want to use robot from simulation run one of the following commands:
```
roslaunch schunk_lwa4p_gazebo lwa4p_wsg50_gazebo.launch
```

# Interfacing with a robot? 

Interface is defined as `a point where two systems, subjects, organizations, etc. meet and interact.`

In order to control robot, we need to interface with it. Interface is provided with topics and services which 
are part of ROS. 

## Send commands 

It is possible to command robot by sending following topics: 
```
control_arm_node/arm/pose
```

Command difference from current pose 
```
control_arm_node/arm/delta_pose
```

## Fundamentals of robotics laboratory exercises 

### First exercise 

Purpose of the first exercise is to get familiar with direct kinematics and simple pick-and-place operation using real robot 
in a simulation. 

Goal is to implement Forward kinematics and correctly use homogenous transformation matrices and matrix multiplication to 
send robot from pose to pose and simulate robot for grasping and retrieving object. 

Script that is used as a starting point is `or_lab_1.py` script. 

### Second exercise 

Purpose of the second exercise is to practice algorithm implementation. In the scope of the second exercise, students 
have to implement Taylor method for planning straight paths in Cartesian space. 

Goal of the exercise is to learn how to manipulate ROS messages and different data types, as well as implementing recursion 
in order to calculate straight path that depends on deviation parameter. 

Script that is used as starting point is `or_lab_2.py` script. 

### Third exercise 

Purpose of the third exercise is to practice programming and algorithm implementation for trajectory planning/execution. 
Third exercise depends on the path planned in the second exercise and students have to implement HoCook trajectory planning 
method. 

Goal of the exercise is to learn clear differentation between path and trajectory and to implement first step 
of the HoCook trajectory planning method. End goal is robot that follows triangular pattern with fixed orientation end effector. 

Script that is used as starting point is `or_lab_3.py` script. 



## Read current arm state 

## Command current arm state 

## RVIZ control 

# ROS control 

Controllers which are used are: 

## Joint state controller 

## Joint trajectory controller 

## Joint position controller

## Joint velocity controller

In order to use real arm, it's neccessary to properly configure 
controllers: 

Controller used for trajectory planning and execution is `jointTrajectoryController`, 
controller used for decoupled joint position moving is `jointPositionController` and 
one used for coupled joint position movement is `jointGroupPositionController`.
