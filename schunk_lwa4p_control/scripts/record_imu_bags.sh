#!/bin/bash

rosbag record /tf \
              /tf_static \
              /imu1/data \
              /imu1/mag \
              /imu2/data \
              /imu2/mag \
              /imu3/data \
              /imu3/mag \
              /control_arm_node/tool/current_pose
