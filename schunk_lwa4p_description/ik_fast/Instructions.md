# How to create IKFast analytical solutions? 

In order to use IKFast inverse kinematics it's neccessary to install [OpenRAVE] and 
generate needed urdf/colada files. In order to do so, install OpenRave with following 
[instructions](https://github.com/crigroup/openrave-installation). 

Generate urdf from urdf.xacro using following command: 
```
rosrun xacro xacro.py <robot_name>.urdf.xacro > <robot_name>.urdf
```

Generate collada file from urdf file using following command: 
```
rosrun collada_urdf urdf_to_collada <robot_name>.urdf <robot_name>.dae
```

Round to 6 decimal places to mitigate possibility of underflow/overflow: 
```
rosrun moveit_kinematics round_collada_numbers.py <robot_name>.dae <rounded_robot_name>.dae <num_decimal_places>
```

Find out info first which links are available: 
```
openrave-robot.py lwa4p_arm_separator.dae --info links
``` 
After that, generate IK cpp: 
```
python ikfast.py --robot=/home/developer/moveit_ws/src/schunk_lwa4p_ros/schunk_lwa4p_description/ik_fast/lwa4p_arm_separator_rounded.dae --iktype=transform6d --baselink=3 --eelink=9 --savefile /home/developer/moveit_ws/src/schunk_lwa4p_ros/schunk_lwa4p_description/ik_fast/ik.cpp 

```

## Official instructions 
[Good official instructions from moveit](http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/ikfast/ikfast_tutorial.html#create-plugin) 



