#!/usr/bin/env python

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
	# T_0_1 = TfromDH(q1+theta1_0, d1, alpha1, a1)
	# T_1_2 = TfromDH(q2+theta2_0, d2, alpha2, a2)
	# T_2_3 = TfromDH(q3+theta3_0, d3, alpha3, a3)
	# T_3_4 = TfromDH(q4+theta4_0, d4, alpha4, a4)
	# T_4_5 = TfromDH(q5+theta5_0, d5, alpha5, a5)
	# T_5_A = TfromDH(q6+theta6_0, d6, alpha6, a6)

	T_0_A = np.dot(T_0_1, np.dot(T_1_2, np.dot(T_2_3, np.dot(T_3_4, np.dot(T_4_5, T_5_A)))))

	return np.asarray([T_0_A[0,3],T_0_A[1,3], T_0_A[2,3]])

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


class OrLab1():
	def __init__(self):
		rospy.init_node("orlab1", anonymous=True, log_level=rospy.DEBUG)
		self.pose_list = []
		self.pose_pub = rospy.Publisher("/control_arm_node/arm/command/pose", Pose, queue_size=10, latch=True)
		self.tf_listener = TransformListener()
		rospy.Subscriber("/lwa4p/joint_states",JointState,self.cb, queue_size=1)
		self.ee_points = []
		self.ee_points_fk = []
		rospy.sleep(0)

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
		# you can set each value in a matrix
		# unlike Matlab, indexing starts at 0
		# first value is T_world_table[0,0]
		# T_world_table[i,j] = value

		## reconstruct T_table_bottle
		# you can also put all 16 values in an array
		T_table_bottle = np.asarray([1, 0, 0, 0,
			                         0, 1, 0, 0,
			                         0, 0, 1, 0,
			                         0, 0, 0, 1])
		# use reshape to create a 4x4 matrix
		T_table_bottle = np.reshape(T_table_bottle, (4,4))

		## reconstruct T_bottle_bottlegrasp


		## reconstruct T_table_brick


		## reconstruct T_brick_brickgrasp

		## calculate grasp poses
		# multiply matrices to get the final pose of the grasp required to pick the object up
		# it's best to use np.dot to multiply matrices T3 = np.dot(T1,T2)

		T_world_brickgrasp = 
		T_world_bottlegrasp = 

		while not rospy.is_shutdown():
			## send the robot through all needed poses by using the matrices you calculated earlier
			self.sendRobotToPose(T_world_brickgrasp)
			self.sendRobotToPose(T_world_bottlegrasp)
			break

		## send robot back to init pose
		self.init_pose()
		## draw graph
		draw(self.ee_points, self.ee_points_fk)


if __name__ == "__main__":
	lab1 = OrLab1()
	lab1.run()
