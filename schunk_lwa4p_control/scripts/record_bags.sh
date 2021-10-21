#!/bin/bash

rosbag record -O $1 /lwa4p/joint_states \
                    /lwa4p/joint_group_position_controller/command \
                    /servo_server/target_pose \
                    /servo_server/delta_joint_cmds \
                    /servo_server/delta_twist_cmds \
                    /servo_server/status \
                    /move_group/goal \
                    /pose_error \
                    /magnetic_estimation \
	            /imu1/data \
                    /imu1/mag \
                    /imu2/data \
                    /imu2/mag \
                    /imu3/data \
                    /imu3/mag \
                    /control_arm_node/tool/current_pose \
                    /magnetic_estimation \
                    /tf \
                    /magnetic_vector1 \
                    /magnetic_vector2 \
                    /magnetic_vector3 \
                    /dynamixel_workbench/dynamixel_state \
                    /dynamixel_workbench/joint_states \
                    /start_time \
                    /end_time \
                    /reached_pose 
 



