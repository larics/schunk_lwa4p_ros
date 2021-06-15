#!/usr/bin/python

import rospy
import sys
from std_msgs.msg import Float64MultiArray, Float64


class jointCtlRepublisher():

    def __init__(self, frequency):

        self.frequency = int(frequency)

        init_sleep_time = 30

        self.recv_cmd = False
        rospy.sleep(init_sleep_time)

        self._initialize_publishers()
        self._initialize_subscribers()

    def _initialize_publishers(self):
        self.q1_cmd = rospy.Publisher("/lwa4p/joint_1_position_controller/command", Float64, queue_size=1)
        self.q2_cmd = rospy.Publisher("/lwa4p/joint_2_position_controller/command", Float64, queue_size=1)
        self.q3_cmd = rospy.Publisher("/lwa4p/joint_3_position_controller/command", Float64, queue_size=1)
        self.q4_cmd = rospy.Publisher("/lwa4p/joint_4_position_controller/command", Float64, queue_size=1)
        self.q5_cmd = rospy.Publisher("/lwa4p/joint_5_position_controller/command", Float64, queue_size=1)
        self.q6_cmd = rospy.Publisher("/lwa4p/joint_6_position_controller/command", Float64, queue_size=1)


    def _initialize_subscribers(self):
        self.current_pose_sub = rospy.Subscriber("/lwa4p/joint_group_position_controller/command",
                                                 Float64MultiArray, callback=self.curr_cmd_cb, queue_size=1)


    def curr_cmd_cb(self, msg):

        self.recv_cmd = True
        self.q1 = msg.data[0]
        self.q2 = msg.data[1]
        self.q3 = msg.data[2]
        self.q4 = msg.data[3]
        self.q5 = msg.data[4]
        self.q6 = msg.data[5]


    def run(self):

        rate = rospy.Rate(self.frequency)


        while not rospy.is_shutdown():
            rate.sleep()
            try:
                if self.recv_cmd:
                    self.q1_cmd.publish(self.q1)
                    self.q2_cmd.publish(self.q2)
                    self.q3_cmd.publish(self.q3)
                    self.q4_cmd.publish(self.q4)
                    self.q5_cmd.publish(self.q5)
                    self.q6_cmd.publish(self.q6)
                else:
                    rospy.loginfo("Haven't recv joint cmd jet")


            except rospy.ROSInterruptException:
                    rospy.logerr("ROS Interrupted")
                    break


if __name__ == "__main__":
    rospy.init_node("joint_ctl_republisher_node")
    mpsP = jointCtlRepublisher(sys.argv[1])
    mpsP.run()