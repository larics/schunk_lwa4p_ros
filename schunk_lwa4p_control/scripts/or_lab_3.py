#!/usr/bin/env python3
# _author_: Filip Zorić; filip.zoric@fer.hr

import rospy
import numpy as np
from math import sqrt

from tf.transformations import quaternion_from_matrix
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped, Pose, Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from schunk_lwa4p_control.srv import getIk
from std_srvs.srv import Trigger

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

def poseToArray(pose):

    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    qx = pose.orientation.x
    qy = pose.orientation.y
    qz = pose.orientation.z
    qw = pose.orientation.w

    return np.asarray([x, y, z, qx, qy, qz, qw])

def arrayToPose(poseArray):

    pose = Pose()
    pose.position.x = poseArray[0]
    pose.position.y = poseArray[1]
    pose.position.z = poseArray[2]
    pose.orientation.x = poseArray[3]
    pose.orientation.y = poseArray[4]
    pose.orientation.z = poseArray[5]
    pose.orientation.w = poseArray[6]

    return pose

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
    T_4_5 = TfromDH(q5+0, 0.0, np.pi/2, 0)
    T_5_A = TfromDH(q6+np.pi, 0.2, 0, 0)
    T_0_A = np.dot(T_0_1, np.dot(T_1_2, np.dot(T_2_3, np.dot(T_3_4, np.dot(T_4_5, T_5_A)))))

    return np.asarray([T_0_A[0,3], T_0_A[1,3], T_0_A[2,3]])

def createGoalPose(x, y, z, qx, qy, qz, qw):
    wanted_pose = Pose()
    wanted_pose.position.x = x;
    wanted_pose.position.y = y;
    wanted_pose.position.z = z;
    wanted_pose.orientation.x = qx;
    wanted_pose.orientation.y = qy;
    wanted_pose.orientation.z = qz;
    wanted_pose.orientation.w = qw;
    return wanted_pose

def draw(points_gazebo, points_fk, cart_points, eps):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    arr_points = np.stack(points_gazebo, axis=0)
    ax.plot(arr_points[:,0],arr_points[:,1],arr_points[:,2], label='Gazebo path')
    fk_points = np.stack(points_fk, axis=0)
    ax.plot(fk_points[:,0],fk_points[:,1],fk_points[:,2], 'r', label='Forward kinematics')
    x_ = [p[0] for p in cart_points]
    y_ = [p[1] for p in cart_points]
    z_ = [p[2] for p in cart_points]
    ax.scatter(x_, y_, z_ , label="Taylor calculated points")
    ax.set_xlabel('x [m]'); ax.set_xlim([-0.3, 0.3])
    ax.set_ylabel('y [m]'); ax.set_ylim([-0.25, 0.25])
    ax.set_zlabel('z [m]'); ax.set_zlim([1.0, 1.8])
    ax.legend(loc='lower right')
    plt.title('End effector path eps={}'.format(eps))
    plt.show()

class OrLab2():
    def __init__(self):
        rospy.init_node("orlab2", anonymous=True, log_level=rospy.INFO)
        self.tf_listener = TransformListener()
        # Init methods
        self._init_subs()
        self._init_pubs()
        self._init_srv_clients()
        # Helper vars
        self.ee_points = []
        self.ee_points_fk = []
        self.current_pose = Pose()
        self.cmd_eps = 0.002
        # Change controller
        rospy.sleep(0)

    def _init_pubs(self):
        self.pose_pub = rospy.Publisher("/control_arm_node/arm/command/pose", Pose, queue_size=10, latch=True)
        self.q_cmd_pub = rospy.Publisher("/lwa4p/joint_group_position_controller/command", Float64MultiArray, queue_size=1)
        rospy.loginfo("Initialized publishers!")

    def _init_subs(self):
        self.pose_sub = rospy.Subscriber("/control_arm_node/tool/current_pose", Pose, self.tool_cb, queue_size=1)
        self.joint_states_sub = rospy.Subscriber("/lwa4p/joint_states",JointState, self.joint_state_cb, queue_size=1)
        rospy.loginfo("Initialized subscribers!")

    def _init_srv_clients(self):
        # Service for fetching IK
        rospy.wait_for_service("/control_arm_node/get_ik")
        self.get_ik_client = rospy.ServiceProxy("/control_arm_node/get_ik", getIk)
        # Service for changing robotic manipulator controller type
        rospy.wait_for_service("/control_arm_node/controllers/start_joint_group_position_controller")
        self.change_controller_client = rospy.ServiceProxy("/control_arm_node/controllers/start_joint_group_position_controller", Trigger)
        rospy.loginfo("Inverse kinematics initialized!")

    def tool_cb(self, msg):

        self.current_pose.position = msg.position
        self.current_pose.orientation = msg.orientation

    def joint_state_cb(self, msg):

        q_s = msg.position
        # Slicing because fingers position is related to gripper
        self.q_curr = msg.position[:-2]
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

    def get_ik(self, wanted_pose):

        if isinstance(wanted_pose, np.ndarray):
            wanted_pose = arrayToPose(wanted_pose)
        try:
            response = self.get_ik_client(wanted_pose, self.current_pose)
            q_ = np.array(response.jointState.position)
            return q_
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: {}".format(e))
            return False

    def execute_cmds(self, q_list):
        # Publish calculated joint values
        for i, q in enumerate(q_list):
            self.execute_cmd(q)
            norm_ = self.norm(q, np.asarray(self.q_curr))
            while norm_ > self.cmd_eps:
                norm_ = self.norm(q, np.asarray(self.q_curr))
                rospy.loginfo_throttle(2, "Executing {} joint cmd".format(i))
                if norm_ < self.cmd_eps:
                    rospy.loginfo("Norm condition satisfied: {}".format(norm_))
                    break
    def execute_cmd(self, q):
        qMsg = Float64MultiArray()
        qMsg.data = q
        self.q_cmd_pub.publish(qMsg)

    def get_ik(self, wanted_pose):

        if isinstance(wanted_pose, np.ndarray):
            wanted_pose = arrayToPose(wanted_pose)
        try:
            response = self.get_ik_client(wanted_pose, self.current_pose)
            q_ = np.array(response.jointState.position)
            return q_
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: {}".format(e))
            return False

    def go_to_pose_taylor(self, goal_pose, eps):
        # Reset saved ee_points and fk points
        self.ee_points = []; self.ee_points_fk = []
        # Start time counting
        start_time = rospy.Time.now().to_sec()
        # calculate taylor points
        points = self.taylor_interpolate_points([poseToArray(self.current_pose), poseToArray(goal_pose)], eps)
        # get ik_poses from calculated_taylor_points (remove first point)
        ik_poses = [self.get_ik(pose) for pose in points[1:]]
        duration = rospy.Time.now().to_sec() - start_time
        # Info print
        rospy.loginfo("Number of points is: {}".format(len(points)))
        rospy.loginfo("Taylor interpolation duration is: {}".format(duration))
        # Execute q_cmds
        self.execute_cmds(ik_poses)
        # draw_path
        draw(self.ee_points, self.ee_points_fk, points, eps)

    def go_to_start_pose(self, start_pose):
        q_start = self.get_ik(start_pose)
        self.execute_cmd(q_start)

    def calc_cartesian_midpoint(self, start_pose, end_pose):
        # 1. zadatak
        # Implementiraj pronalazenje medjutocke u prostoru alata
        # Zbog jednostavnosti zadrzi orijentaciju pocetne tocke
        # Metoda vraca np.array medjutocke u prostoru alata
        pass

    def calc_joint_midpoint(self, q_start, q_end):
        # 2. zadatak
        # Implementiraj pronalazenje medjutocke u prostoru zglobova
        # Metoda vraca np.array medjutocke u prostoru zglobova
        pass

    def norm(self, v1, v2):
        # 3. zadatak
        # Implementiraj pronalazenje Frobeniusove norme
        # za jednodimenzionalne vektore proizvoljne duljine
        # metoda vraca skalarnu vrijednost norme
        pass

    def taylor_interpolate_point(self, start_pose, end_pose, epsilon):
        # 4. zadatak
        # Implementiraj Taylorovu metodu odstupanja od tocke koristeci predefinirane argumente
        # Ukoliko je odstupanje (epsilon) manje od zeljenog - metoda treba vratiti listu [start_pose, p_wM, end_pose]
        # Ukoliko odstupanje nije manje od zeljenog, metoda vraca False
        pass

    def taylor_interpolate_points(self, poses_list, epsilon):
        # 5. zadatak
        # U ovoj metodi potrebno je implementirati algoritam Taylorovog odstupanja u cijelosti
        # Metoda kao argumente prima listu poza kroz koje zelimo proci s alatom
        # Radi jednostavnosti implementacije, orijentacija alata u svim tockama mora biti ista
        # Ako je zadano odstupanje epsilon zadovoljeno, metoda vraca listu tocaka u prostoru alata kroz koje robot treba proci
        # Ako odstupanje nije zadovoljeno, metoda se rekurzivno poziva
        pass

    def run(self):
        # Initialize starting pose
        self.init_pose()
        # Sleep for 5. seconds
        rospy.sleep(10.0)
        # Initialize goal pose
        start_pose = createGoalPose(0., 0.25, 1.75, -0.5, 0.5, 0.5, 0.5)
        wanted_pose1 = createGoalPose(0.25, 0.25, 1.4, -0.5, 0.5, 0.5, 0.5)
        wanted_pose2 = createGoalPose(0.5, 0.25, 1.4, -0.5, 0.5, 0.5, 0.5)

        self.change_controller_client.call()
        while not rospy.is_shutdown():
            test = True
            if test:
                eps = 0.75
                self.go_to_pose_taylor(wanted_pose1, eps)
                rospy.sleep(1.0)

                self.go_to_start_pose(start_pose)
                rospy.sleep(1.0)

                #eps = 0.5
                #self.go_to_pose_taylor(wanted_pose1, eps)
                #rospy.sleep(1.0)

                #self.go_to_start_pose(start_pose)
                #rospy.sleep(3.0)

                #eps = 0.25
                #self.go_to_pose_taylor(wanted_pose1, eps)
                #rospy.sleep(1.0)

                #self.go_to_start_pose(start_pose)
                #rospy.sleep(3.0)

                #eps = 0.1
                #self.go_to_pose_taylor(wanted_pose1, eps)
                #rospy.sleep(1.0)

                #self.go_to_start_pose(start_pose)
                #rospy.sleep(3.0)

                #eps = 0.05
                #self.go_to_pose_taylor(wanted_pose1, eps)
                #rospy.sleep(1.0)

                #self.go_to_start_pose(start_pose)
                #rospy.sleep(3.0)

                #eps = 0.02
                #self.go_to_pose_taylor(wanted_pose1, eps)
                #rospy.sleep(3.0)

                #eps = 0.5
                #self.go_to_pose_taylor(wanted_pose2, eps)
                #rospy.sleep(1.0)

                #self.go_to_start_pose(start_pose)
                #rospy.sleep(3.0)

                #eps = 0.5
                #self.go_to_pose_taylor(wanted_pose2, eps)
                #rospy.sleep(1.0)

                #self.go_to_start_pose(wanted_pose1)
                #rospy.sleep(3.0)

                #eps = 0.2
                #self.go_to_pose_taylor(wanted_pose2, eps)
                #rospy.sleep(1.0)

                break

        exit()


if __name__ == "__main__":
    lab2 = OrLab2()
    lab2.run()


