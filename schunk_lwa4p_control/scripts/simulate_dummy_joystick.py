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
        self.final_pose.position.x = 0.2; self.final_pose.position.y = 0.; self.final_pose.position.z = 1.1;
        self.final_pose.orientation.x = 0; self.final_pose.orientation.y = 0; self.final_pose.orientation.z = -1; self.final_pose.orientation.w = 0;

        #init_sleep_time = 5
        #rospy.sleep(init_sleep_time)

        self._initialize_publishers()
        rospy.loginfo("[DummyJoy] Initialized publishers...")
        self._initialize_subscribers()
        rospy.loginfo("[DummyJoy] Initialized subscribers...")

        self.first_trig_reciv = False
        self.publish_ = False

    def _initialize_publishers(self):
        self.cmd_pose_pub = rospy.Publisher("/sim_joy/target_pose", PoseStamped, queue_size=1)

    def _initialize_subscribers(self):
        self.start_trigger = rospy.Subscriber("/sim_joy/start_servo_sim", Bool, callback=self.start_trigger_cb)

    def start_trigger_cb(self, data):

        if not self.first_trig_reciv:
            self.first_trig_reciv = True

        self.publish_ = data.data

    def create_arrays(self, x_max, y_max, z_max, qx_max, qy_max, qz_max, qw_max, num_measurements):

        x_array = np.linspace(0, x_max, num_measurements);
        y_array = np.linspace(0, y_max, num_measurements);
        z_array = np.linspace(0, z_max, num_measurements);
        qx_array = np.linspace(0, qx_max, num_measurements);
        qy_array = np.linspace(0, qy_max, num_measurements);
        qz_array = np.linspace(0, qz_max, num_measurements);
        qw_array = np.linspace(0, qw_max, num_measurements);

        return x_array, y_array, z_array, qx_array, qy_array, qz_array, qw_array

    def run(self):

        rate = rospy.Rate(self.frequency)

        i = 0

        num_meas = 10000
        x_, y_, z_, qx_, qy_, qz_, qw_ = self.create_arrays(self.final_pose.position.x, self.final_pose.position.y, self.final_pose.position.z,
                                                            self.final_pose.orientation.x, self.final_pose.orientation.y,
                                                            self.final_pose.orientation.z, self.final_pose.orientation.w,
                                                            num_measurements=num_meas);


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