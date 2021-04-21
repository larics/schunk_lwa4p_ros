#!/usr/bin/python

import rospy
import sys
import numpy as np
from math import sqrt
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import Bool
from moveit_msgs.msg import MoveGroupActionFeedback
import copy
import random

class ServoPosePublisher():

    def __init__(self, frequency):

        self.frequency = int(frequency)
        self.current_pose = Pose()
        # Set final pose
        self.final_pose = Pose()
        self.final_pose.position.x = -0.4; self.final_pose.position.y = 0.20; self.final_pose.position.z = 1.0;
        self.final_pose.orientation.x = 0.0263204258001; self.final_pose.orientation.y = 0.0171458553782;
        self.final_pose.orientation.z = -0.435398218684; self.final_pose.orientation.w = 0.899689749856;

        #init_sleep_time = 5
        #rospy.sleep(init_sleep_time)

        self._initialize_publishers()
        rospy.loginfo("[SimServoJoy] Initialized publishers...")
        self._initialize_subscribers()
        rospy.loginfo("[SimServoJoy] Initialized subscribers...")

        self.first_trig_reciv = False
        self.first_curr_pose_reciv = True
        self.publish_ = False

    def _initialize_publishers(self):
        self.cmd_pose_pub = rospy.Publisher("/sim_joy/target_pose", PoseStamped, queue_size=1)

    def _initialize_subscribers(self):
        self.start_trigger = rospy.Subscriber("/sim_joy/start_servo_sim", Bool, callback=self.start_trigger_cb)
        self.current_ee_pose = rospy.Subscriber("/control_arm_node/tool/current_pose", Pose, callback=self.curr_ee_pose_cb)

    def start_trigger_cb(self, msg):

        if not self.first_trig_reciv:
            self.first_trig_reciv = True

        self.publish_ = msg.data

    def curr_ee_pose_cb(self, msg):

        if not self.first_curr_pose_reciv:
            self.first_curr_pose_reciv = True

        self.current_tool_pose = Pose()
        self.current_tool_pose.position.x = msg.position.x;
        self.current_tool_pose.position.y = msg.position.y;
        self.current_tool_pose.position.z = msg.position.z;
        self.current_tool_pose.orientation.x = msg.orientation.x;
        self.current_tool_pose.orientation.y = msg.orientation.y;
        self.current_tool_pose.orientation.z = msg.orientation.z;
        self.current_tool_pose.orientation.w = msg.orientation.w;


    def create_arrays(self, x_max, y_max, z_max, qx_max, qy_max, qz_max, qw_max, num_measurements = 10000, start_pose=None):

        if not start_pose:
            start_val_x, start_val_y, start_val_z, start_val_qx, start_val_qy, start_val_qz, start_val_qw = 0, 0, 0, 0, 0, 0, 0
        else:
            start_val_x = start_pose.position.x
            start_val_y = start_pose.position.y
            start_val_z = start_pose.position.z
            start_val_qx = start_pose.orientation.x
            start_val_qy = start_pose.orientation.y
            start_val_qz = start_pose.orientation.z
            start_val_qw = start_pose.orientation.w

        x_array = np.linspace(start_val_x, x_max, num_measurements)
        y_array = np.linspace(start_val_y, y_max, num_measurements)
        z_array = np.linspace(start_val_z, z_max, num_measurements)
        qx_array = np.linspace(start_val_qx, qx_max, num_measurements)
        qy_array = np.linspace(start_val_qy, qy_max, num_measurements)
        qz_array = np.linspace(start_val_qz, qz_max, num_measurements)
        qw_array = np.linspace(start_val_qw, qw_max, num_measurements)

        return x_array, y_array, z_array, qx_array, qy_array, qz_array, qw_array

    def run(self):

        rate = rospy.Rate(self.frequency)
        i = 0

        while not self.first_curr_pose_reciv:
            rate.sleep()
            rospy.loginfo("Waiting for ee pose...")

        num_meas = 10000
        x_, y_, z_, qx_, qy_, qz_, qw_ = self.create_arrays(self.final_pose.position.x, self.final_pose.position.y, self.final_pose.position.z,
                                                            self.final_pose.orientation.x, self.final_pose.orientation.y,
                                                            self.final_pose.orientation.z, self.final_pose.orientation.w,
                                                            num_measurements=num_meas);

        rospy.logdebug("[SimServoJoy] x_min: {}\tx_max: \t".format(round(np.min(x_), 2), round(np.max(x_), 2)))
        rospy.logdebug("[SimServoJoy] y_min: {}\ty_max: \t".format(round(np.min(y_), 2), round(np.max(y_), 2)))
        rospy.logdebug("[SimServoJoy] z_min: {}\tz_max: \t".format(round(np.min(z_), 2), round(np.max(z_), 2)))


        while not rospy.is_shutdown():
            try:
                if self.publish_:
                    for i in range(0, num_meas):
                        cmd_pose = PoseStamped()
                        # TODO: Check which differences are based on different links (base_link, lwa4p_link6)
                        cmd_pose.header.frame_id = "base_link"
                        cmd_pose.pose.position.x = x_[i]; cmd_pose.pose.position.y = y_[i]; cmd_pose.pose.position.z = z_[i];
                        cmd_pose.pose.orientation.x = qx_[i]; cmd_pose.pose.orientation.y = qy_[i];
                        cmd_pose.pose.orientation.z = qz_[i]; cmd_pose.pose.orientation.w = qw_[i];

                        self.cmd_pose_pub.publish(cmd_pose)

                        rate.sleep()

            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupted")
                break


if __name__ == "__main__":
    rospy.init_node("magnetic_pose_publisher_sim")
    sPP = ServoPosePublisher(sys.argv[1])
    sPP.run()