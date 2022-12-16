#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point
import numpy as np
from tf.transformations import quaternion_from_matrix
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def poseFromMatrix(matrix):
	goal_pose = Pose()
	quat = quaternion_from_matrix(matrix)

	goal_pose.position.x = matrix[0,3]
	goal_pose.position.y = matrix[1,3]
	goal_pose.position.z = matrix[2,3]
	goal_pose.orientation.x = quat[0]
	goal_pose.orientation.y = quat[1]
	goal_pose.orientation.z = quat[2]
	goal_pose.orientation.w = quat[3]

	return goal_pose

def TfromDH(theta, d, alpha, a):
	T = np.eye(4)
	T[0,0] = np.cos(theta)
	T[0,1] = -np.sin(theta)*np.cos(alpha)
	T[0,2] = np.sin(theta)*np.sin(alpha)
	T[0,3] = a*np.cos(theta)
	T[1,0] = np.sin(theta)
	T[1,1] = np.cos(theta)*np.cos(alpha)
	T[1,2] = -np.cos(theta)*np.sin(alpha)
	T[1,3] = a*np.sin(theta)
	T[2,0] = 0
	T[2,1] = np.sin(alpha)
	T[2,2] = np.cos(alpha)
	T[2,3] = d
	return T

def forwardKinematics(q_s):
	q1 = q_s[0]
	q2 = q_s[1]
	q3 = q_s[2]
	q4 = q_s[3]
	q5 = q_s[4]
	q6 = q_s[5]

	## insert your kinematic parameters here
	T_0_1 = TfromDH(q1+0, 1.15, np.pi/2, 0)
	T_1_2 = TfromDH(q2+np.pi/2, 0, np.pi, 0.35)
	T_2_3 = TfromDH(q3+np.pi/2, 0, np.pi/2, 0)
	T_3_4 = TfromDH(q4+0, 0.305, -np.pi/2, 0)
	T_4_5 = TfromDH(q5+0, 0.005263, np.pi/2, 0)
	T_5_A = TfromDH(q6+np.pi, 0.2, 0, 0)
	T_0_A = np.dot(T_0_1, np.dot(T_1_2, np.dot(T_2_3, np.dot(T_3_4, np.dot(T_4_5, T_5_A)))))

	return np.asarray([T_0_A[0,3],T_0_A[1,3], T_0_A[2,3]])

def init_pose(self):
	init_pose_goal = Pose()
	init_pose_goal.position.x = 0.
	init_pose_goal.position.y = 0.25
	init_pose_goal.position.z = 1.75
	init_pose_goal.orientation.x = -0.5
	init_pose_goal.orientation.y = 0.5
	init_pose_goal.orientation.z = 0.5
	init_pose_goal.orientation.w = 0.5
	self.pose_pub.publish(init_pose_goal)
	rospy.sleep(5)

def draw(points_gazebo, points_fk):
	fig = plt.figure()
	ax = fig.add_subplot(111, projection='3d')
	arr_points = np.stack(points_gazebo, axis=0)
	ax.plot(arr_points[:,0],arr_points[:,1],arr_points[:,2], label='Gazebo path')
	fk_points = np.stack(points_fk, axis=0)
	ax.plot(fk_points[:,0],fk_points[:,1],fk_points[:,2], 'r', label='Forward kinematics')
	ax.set_xlabel('x [m]')
	ax.set_ylabel('y [m]')
	ax.set_zlabel('z [m]')
	ax.legend(loc='lower right')
	plt.title('End effector path')
	plt.show()


class OrLab2():
	def __init__(self):
		rospy.init_node("orlab1", anonymous=True, log_level=rospy.DEBUG)
		self.pose_list = []
		self.tf_listener = TransformListener()
		self._init_subs()
		self._init_pubs()
		self.ee_points = []
		self.ee_points_fk = []
		rospy.sleep(0)

	def _init_pubs(self):
		self.pose_pub = rospy.Publisher("/control_arm_node/arm/command/pose", Pose, queue_size=10, latch=True)
		rospy.loginfo("Initialize publishers!")

	def _init_subs(self):
		self.pose_sub = rospy.Subscriber("/control_arm_node/tool/current_pose", Pose, self.tool_cb, queue_size=1)
		self.joint_states_sub = rospy.Subscriber("/lwa4p/joint_states",JointState, self.cb, queue_size=1)
		rospy.loginfo("Initialized subscribers!")


	def tool_cb(self, msg):

		self.current_pose.pose.position = msg.pose.position
		self.current_pose.pose.orientation = msg.pose.orientation



	def get_cartesian_mid_pose(self, start_pose, end_pose):

		mid_pose = start_pose + end_pose/2

		return mid_pose

	def check_pose_norm(self, pose1, pose2):

		np.linalg.norm(pose1, )

	def get_ik(self, pose):

		# TODO: Think how to get inverse kinematics for it





	def cb(self, msg):
		q_s = msg.position
		try:
			(t,q) = self.tf_listener.lookupTransform("world", "wsg50_center", rospy.Duration(0))
		except (LookupException, ConnectivityException, ExtrapolationException):
			return
		if len(self.ee_points) == 0:
			self.ee_points.append(np.asarray([t[0], t[1], t[2]]))
			self.ee_points_fk.append(forwardKinematics(q_s))
		else:
			new_p = np.asarray([t[0], t[1], t[2]])
			last_p = self.ee_points[-1]
			if np.linalg.norm(new_p-last_p) > 0.0001:
				self.ee_points.append(new_p)
				self.ee_points_fk.append(forwardKinematics(q_s))

	def sendRobotToPose(self, matrix):
		self.pose_pub.publish(poseFromMatrix(matrix))
		rospy.sleep(5)


	def run(self):
		self.init_pose()

		## construct T_world_table
		# this creates a 4x4 identity matrix
		T_world_table = np.eye(4)
		# T_world_table[i,j] = value
		T_world_table[0,3] = -1.0
		T_world_table[1,3] = 0.25
		T_world_table[2,3] = 1.0


		## reconstruct T_table_bottle
		T_table_bottle = np.asarray([0, -1, 0, 1.25,
									 0, 0, 1, 0.2,
									 -1, 0, 0, 0.11,
									 0, 0, 0, 1])
		T_table_bottle = np.reshape(T_table_bottle, (4,4))

		T_table_bottle2 = np.asarray([0, -1, 0, 1.25,
									  0, 0, 1, 0.2,
									  -1, 0, 0, 0.27,
									  0, 0, 0, 1])
		T_table_bottle2 = np.reshape(T_table_bottle2, (4,4))

		## reconstruct T_bottle_bottlegrasp

		## reconstruct T_table_brick
		T_table_brick = np.asarray([ 1, 0, 0, 0.6,
									 0, -1, 0, 0.05,
									 0, 0, -1, 0.15,
									 0, 0, 0, 1])
		T_table_brick = np.reshape(T_table_brick, (4,4))

		T_table_brick2 = np.asarray([ 1, 0, 0, 0.6,
									  0, -1, 0, 0.05,
									  0, 0, -1, 0.3,
									  0, 0, 0, 1])
		T_table_brick2 = np.reshape(T_table_brick2, (4,4))

		## reconstruct T_brick_brickgrasp

		## calculate grasp poses
		T_world_brickapp = np.dot(T_world_table, T_table_brick2)
		T_world_bottleapp = np.dot(T_world_table, T_table_bottle2)
		T_world_brickgrasp = np.dot(T_world_table, T_table_brick)
		T_world_bottlegrasp = np.dot(T_world_table, T_table_bottle)

		# self.pose_pub.publish(poseFromMatrix(T_world_brickgrasp))
		# rospy.sleep(5)
		# self.pose_pub.publish(poseFromMatrix(T_world_bottlegrasp))

		while not rospy.is_shutdown():
			self.sendRobotToPose(T_world_brickapp)
			self.sendRobotToPose(T_world_brickgrasp)
			self.sendRobotToPose(T_world_bottleapp)
			self.sendRobotToPose(T_world_bottlegrasp)

			print(T_world_brickgrasp)
			print(T_world_bottlegrasp)
			break
		self.init_pose()
		## send robot back to init pose
		self.init_pose()
		## draw graph
		draw(self.ee_points, self.ee_points_fk)


if __name__ == "__main__":
	lab1 = OrLab1()
	lab1.run()