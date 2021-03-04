#!/usr/bin/python

import rospy
import sys
import numpy as np
from math import sqrt
from geometry_msgs.msg import Pose
from moveit_msgs.msg import MoveGroupActionFeedback

class MangeticPoseSimulatorPublisher():

    def __init__(self, frequency):

        self.frequency = int(frequency)
        self.current_pose = Pose()
        self.final_pose = MangeticPoseSimulatorPublisher.set_final_pose(0.2, 0., 1.2, 0, 0, -1, 0)
        self.delta_control = False
        self.first_trajectory_published = False
        self.trajectory_execution_state = "NONE"

        init_sleep_time = 30
        rospy.sleep(init_sleep_time)

        self._initialize_publishers()
        self._initialize_subscribers()

    def _initialize_publishers(self):
        self.cmd_pose_pub = rospy.Publisher("/tool/cmd_pose", Pose, queue_size=1)

        if self.delta_control:
            self.cmd_delta_pose_pub = rospy.Publisher("/control_arm_node/arm/command/delta_pose", Pose, queue_size=1)

    def _initialize_subscribers(self):
        self.current_pose_sub = rospy.Subscriber("/control_arm_node/tool/current_pose", Pose, callback=self.curr_pose_cb)

        if self.delta_control:
            self.move_group_fb_sub = rospy.Subscriber("/move_group/feedback", MoveGroupActionFeedback, callback=self.move_group_fb_cb)
            #self.move_group_status_sub = rospy.Subscriber("/move_group/status", )

    def curr_pose_cb(self, msg):

        self.current_pose.position.x = msg.position.x
        self.current_pose.position.y = msg.position.y
        self.current_pose.position.z = msg.position.z
        self.current_pose.orientation.x = msg.orientation.x
        self.current_pose.orientation.y = msg.orientation.y
        self.current_pose.orientation.z = msg.orientation.z
        self.current_pose.orientation.w = msg.orientation.w

    def create_white_noise(self, mean, std, num_samples):

        return np.random.normal(mean, std, size=num_samples)

    def move_group_fb_cb(self, msg):

        self.trajectory_execution_state = msg.feedback.state
        rospy.loginfo("Trajectory execution state is: {}".format(self.trajectory_execution_state))

    #def move_group_status_cb(self, msg):
    #actionlibs
    #    self.trajectory_execution_state = msg.status_list.status


    def check_position_delta(self, curr_pose, wanted_pose):
        """
        Check position delta between current pose and wanted pose
        :param curr_pose:
        :param wanted_pose:
        :return:
        """

        current_position = curr_pose.position
        wanted_position = wanted_pose.position

        delta_x = wanted_position.x - current_position.x
        delta_y = wanted_position.y - current_position.y
        delta_z = wanted_position.z - current_position.z

        return delta_x, delta_y, delta_z

    def create_delta_trajectory_per_cartesian_axis(self, delta_axis, num_steps):
        """
        Create uniform delta per each cartesian axis
        :param delta_axis:
        :param num_steps:
        :return:
        """

        delta_axis_step_increment = delta_axis/num_steps

        return [delta_axis_step_increment] * num_steps

    @staticmethod
    def check_dist(pose1, pose2):

        position1 = pose1.position
        position2 = pose2.position

        x_dist = (position1.x - position2.x) ** 2
        y_dist = (position1.y - position2.y) ** 2
        z_dist = (position1.z - position2.z) ** 2

        dist = sqrt(x_dist + y_dist + z_dist)

        return dist

    @staticmethod
    def set_final_pose(x, y, z, qx, qy, qz, qw):

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        return pose

    def run(self):

        rate = rospy.Rate(self.frequency)

        i = 0


        while not rospy.is_shutdown():
            rate.sleep()
            try:

                # Check Euclidean distance
                dist = self.check_dist(self.current_pose, self.final_pose)
                # rospy.loginfo("Current distance is: {}".format(dist))

                

                ### ---> Delta control <---- ####
                if self.delta_control:

                    delta_axis_x, delta_axis_y, delta_axis_z = self.check_position_delta(self.current_pose, self.final_pose)

                    # Create delta trajectories
                    num_pts = 15
                    delta_x = self.create_delta_trajectory_per_cartesian_axis(delta_axis_x, num_pts)
                    delta_y = self.create_delta_trajectory_per_cartesian_axis(delta_axis_y, num_pts)
                    delta_z = self.create_delta_trajectory_per_cartesian_axis(delta_axis_z, num_pts)

                    #i += 1

                    for dx, dy, dz in zip(delta_x, delta_y, delta_z):
                            cmd_delta_pose = Pose()
                            cmd_delta_pose.position.x = dx
                            cmd_delta_pose.position.y = dy
                            cmd_delta_pose.position.z = dz
                            cmd_delta_pose.orientation.x, cmd_delta_pose.orientation.y, cmd_delta_pose.orientation.z, cmd_delta_pose.orientation.w = 0, 0, 0, 0

                    rospy.loginfo("____DeLtA____")

                    # Check execution state move_group
                    if not self.first_trajectory_published:
                        self.cmd_delta_pose_pub.publish(cmd_delta_pose)
                        self.first_trajectory_published = True

                    if self.trajectory_execution_state == "IDLE":
                        self.cmd_delta_pose_pub.publish(cmd_delta_pose)

            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupted")
                break


if __name__ == "__main__":
    rospy.init_node("magnetic_pose_publisher_sim")
    mpsP = MangeticPoseSimulatorPublisher(sys.argv[1])
    mpsP.run()