#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import DisplayTrajectory
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from moveit_commander.conversions import pose_to_list
from termcolor import colored, cprint

class ControlArm():

    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface', anonymous=True)
        timeout_s = 60
        rospy.sleep(timeout_s)
        self.rate = rospy.Rate(20)

        # Instantiate a RobotCommander object (outer-level interface to the robot)
        self.robot = moveit_commander.RobotCommander()

        # Instantiate a PlanningSceneInterface object (Interface to the world surrounding the robot)
        self.scene = moveit_commander.PlanningSceneInterface()

        group_name = "arm"
        self.group = moveit_commander.MoveGroupCommander(group_name)
        #print(dir(self.group))

        # Initialize message for publishing pose
        self.commanded_pose = PoseStamped()
        self.interactive_planning = True
        self.received_commanded_pose = False

        self._initialize_publishers()
        self._initialize_subscribers()

        rospy.loginfo(colored("ControlArm Initialization successful!", "red"))

        self.plan_changed = True

    ## Basic ROS methods
    def _initialize_subscribers(self):

        self.command_pose_sub = rospy.Subscriber("arm/command/pose",
                                                  PoseStamped,
                                                  callback=self.pose_cmd_cb)

    def _initialize_publishers(self): 

        self.display_trajectory_pub = rospy.Publisher('/move_group/display_planned_path', 
                                                       moveit_msgs.msg.DisplayTrajectory, 
                                                       queue_size=20)

    ## Callbacks
    def pose_cmd_cb(self, msg):
        
        rospy.loginfo(colored("Received pose command!", "green"))
        self.received_commanded_pose = True
        poseMsg = PoseStamped()
        poseMsg.header = msg.header
        poseMsg.pose = msg.pose
        self.commanded_pose = poseMsg
        self.plan_changed = True


    def get_basic_info(self):
        # Get the name of the reference frame for this robot:
        planning_frame = self.group.get_planning_frame()
        print("============ Reference frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group 
        eef_link = self.group.get_end_effector_link()
        print("============ End effector: %s" % eef_link)

        # We can get a list of all the groups in the robot: 
        group_names = self.robot.get_group_names()
        print("============ Robot Groups: ", self.robot.get_group_names())

        # Sometimes for debugging it's useful to print the entire state of the robot 
        print("============ Printing robot state")
        print(self.robot.get_current_state())
        print("============")

    def plan_pose_goal(self):
        
        print("Planned pose goal!")
        if  self.interactive_planning and self.received_commanded_pose:
            pose_goal = self.commanded_pose
        else: 
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.orientation.w = 1.0
            pose_goal.orientation.z = 0
            pose_goal.orientation.y = 0
            pose_goal.orientation.x = 0
            pose_goal.position.x = 0.4 
            pose_goal.position.y = 0.2
            pose_goal.position.z = 1 

        self.group.set_pose_target(pose_goal)

    def call_planner(self):        
        plan = self.group.go(wait=True)
        # Calling stop() ensures that there is no residual movement 
        self.group.stop()
        # It is always goot to clear targets after planning with poses
        self.group.clear_pose_targets()

        return plan


    def display_trajectory(self, plan):
        
        # Create DisplayTrajectory ROS msg
        displayTrajectoryMsg = DisplayTrajectory()
        displayTrajectoryMsg.trajectory_start = self.robot.get_current_state()
        displayTrajectoryMsg.trajectory.append(plan)

        # Publish
        self.display_trajectory_pub.publish(displayTrajectoryMsg)

    def run(self):
        rospy.sleep(20)

        while not rospy.is_shutdown():
            rospy.loginfo(colored("Planning pose goal", "red"))
            self.rate.sleep()
            #self.get_basic_info()
            if self.plan_changed:
                self.plan_pose_goal()
                plan = self.call_planner()
                #self.display_trajectory(plan)
                self.plan_changed = False

                #self.execute_planned_path(plan)
                #self.plan_changed = False


if __name__ == "__main__":

    ctl_arm = ControlArm()
    ctl_arm.run()

     