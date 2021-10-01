#!/bin/bash

rosbag record /lwa4p/joint_states \
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
              /tf



