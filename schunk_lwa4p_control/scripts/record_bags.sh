#!/bin/bash

rosbag record /lwa4p/joint_states \
              /lwa4p/joint_group_position_controller/command \
              /servo_server/target_pose \
              /servo_server/delta_joint_cmds \
              /servo_server/delta_twist_cmds 

