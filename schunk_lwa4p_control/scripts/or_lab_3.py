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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

np.printoptions(precision=3, suppress=True)

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
    wanted_pose.position.x = x
    wanted_pose.position.y = y
    wanted_pose.position.z = z
    wanted_pose.orientation.x = qx
    wanted_pose.orientation.y = qy
    wanted_pose.orientation.z = qz
    wanted_pose.orientation.w = qw
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

class OrLab3():
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
        np.printoptions(precision=3, suppress=True)

    def _init_pubs(self):
        self.pose_pub = rospy.Publisher("/control_arm_node/arm/command/pose", Pose, queue_size=10, latch=True)
        self.q_cmd_pub = rospy.Publisher("/lwa4p/joint_group_position_controller/command", Float64MultiArray, queue_size=1)
        self.trajectory_pub = rospy.Publisher("/lwa4p/arm_controller/command", JointTrajectory, queue_size=1)
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
        self.change_to_pos_client = rospy.ServiceProxy("/control_arm_node/controllers/start_joint_group_position_controller", Trigger)
        rospy.wait_for_service("/control_arm_node/controllers/start_joint_trajectory_controller")
        self.change_to_traj_client = rospy.ServiceProxy("/control_arm_node/controllers/start_joint_trajectory_controller", Trigger)
        rospy.loginfo("Initialized services/clients!")

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

    def createTrajectory(self, q, dq, t):

        joint_names =  ["lwa4p_joint1", "lwa4p_joint2", "lwa4p_joint3", "lwa4p_joint4", "lwa4p_joint5", "lwa4p_joint6"]

        trajectoryMsg = JointTrajectory()
        trajectoryMsg.joint_names = joint_names

        dq = list(dq.T)
        i = 0
        print("t is: {}".format(t))
        for k, (q, dq) in enumerate(zip(q, dq)):
            try:
                i += t[k]
                t_ = rospy.Time.from_sec(i)
            except Exception as e:
                t_ = rospy.Time.from_sec(np.ceil(i))
            trajectoryPoint = JointTrajectoryPoint()
            trajectoryPoint.positions = q
            trajectoryPoint.velocities = dq
            trajectoryPoint.time_from_start.secs = t_.secs
            trajectoryPoint.time_from_start.nsecs = t_.nsecs
            #trajectoryPoint.time_from_start.nsecs = (k*inc * 1e9)
            trajectoryMsg.points.append(trajectoryPoint)

        print(trajectoryMsg)

        return trajectoryMsg

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

    def go_to_start_pose(self, start_pose):
        q_start = self.get_ik(start_pose)
        self.execute_cmd(q_start)

    def sendRobotToPose(self, matrix):
        self.pose_pub.publish(poseFromMatrix(matrix))
        rospy.sleep(5)
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

    def getBfirstSeg(self, q, dq, t):

        T = np.zeros((4, 5))
        T[0, 0] = 1;            T[0, 3] = -4/t[0]**3;   T[0, 4] = 3/t[0]**4
        T[1, 3] = 4/t[0]**3;    T[1, 4] = -3/t[0]**4;
        T[3, 3] = -1/t[0]**2;   T[3, 4] = 1/t[0]**3;

        Q = np.array((q[0], q[1], dq[:, 0], dq[:, 1])).T


        BfirstSeg = np.matmul(Q, T) # 6 x 5 dimensions

        return BfirstSeg

    def getBanySeg(self, q, dq, t, k):

        T = np.zeros((4, 5))
        T[0, 0] = 1;            T[0, 3] = -3/t[k]**2;   T[0, 4] = 2/t[k]**3;
        T[1, 3] = 3/t[k]**2;    T[1, 4] = -2/t[k]**3;
        T[2, 1] = 1;            T[2, 2] = -2/t[k];      T[2, 3] = 1/t[k]**2;
        T[3, 2] = -1/t[k];      T[3, 3] = 1/t[k]**2;

        Q = np.array((q[k-1], q[k], dq[:, k-1], dq[:, k])).T

        BkSeg = np.matmul(Q, T)

        return BkSeg

    def getBLastSeg(self, q, dq, t):

        T = np.zeros((4, 5))
        T[0, 0] = 1;            T[0, 2] = -6/t[-1]**2;  T[0, 3] = 8/t[-1]**2; T[0, 4] = -3/t[-1]**4;
        T[1, 2] = 6/t[-1]**2;   T[1, 3] = -8/t[-1]**3;  T[1, 4] = 3/t[-1]**4;
        T[2, 1] = 1;            T[2, 2] = -3/t[-1];     T[2, 3] = 3/t[-1]**2; T[2, 4] = -1/t[-1]**3;

        Q = np.array((q[-2], q[-1], dq[: ,-2], dq[:, -1])).T

        BlastSeg = np.matmul(Q, T)

        return BlastSeg

    def getMaxSpeed(self, B, t):

        q_max = B[:, 1] + 2*B[:, 2]*t + 3*B[:, 3]*t**2

        return q_max

    def getMaxAcc(self, B, t):

        q_dot_max = 2*B[:, 2] + 6*B[:, 3]*t

        return q_dot_max

    # 2. Zadaća implementirana
    def calc_cartesian_midpoint(self, start_pose, end_pose):

        # Calculate position average
        x = (start_pose.position.x + end_pose.position.x)/2
        y = (start_pose.position.y + end_pose.position.y)/2
        z = (start_pose.position.z + end_pose.position.z)/2
        # Keep orientation the same as in starting points
        qx = (start_pose.orientation.x)
        qy = (start_pose.orientation.y)
        qz = (start_pose.orientation.z)
        qw = (start_pose.orientation.w)

        return np.asarray([x, y, z, qx, qy, qz, qw])

    def calc_joint_midpoint(self, start_joint, end_joint):
        return (start_joint + end_joint)/2

    def norm(self, q_cmd, q_curr):
        norm = np.sqrt(np.sum((q_cmd - q_curr)**2))
        return norm

    def taylor_interpolate_point(self, start_pose, end_pose, epsilon):
        # Get q0, q1
        q0 = self.get_ik(start_pose)
        q1 = self.get_ik(end_pose)
        # get qm
        qm = self.calc_joint_midpoint(q0, q1)
        # calculate w_m
        p_wm = forwardKinematics(qm)
        # calculate w_M
        p_wM = self.calc_cartesian_midpoint(start_pose, end_pose)
        # calcukate norm between w_m and w_M
        if (self.norm(p_wm, p_wM) < epsilon):
            rospy.loginfo("Taylor interpolation finished")
            return [start_pose, p_wM, end_pose]
        else:
            rospy.loginfo("Taylor interpolation, condition not satisfied")
            return False

    def taylor_interpolate_points(self, poses_list, epsilon):
        # IK/FK Poses
        ik_poses = []
        calc_epsilons = []
        cartesian_points = []
        added_joints = []
        added_points = []

        for i, pose in enumerate(poses_list):
            if i == 0:
                cartesian_points.append(pose)
            if i > 0 and i < len(poses_list):
                avg_pt = self.calc_cartesian_midpoint(arrayToPose(poses_list[i - 1]), arrayToPose(pose))
                avg_joint = self.calc_joint_midpoint(self.get_ik(poses_list[i - 1]), self.get_ik(pose))
                cartesian_points.append(avg_pt)
                added_points.append(avg_pt)
                added_joints.append(avg_joint)
            if i == len(poses_list) - 1:
                cartesian_points.append(pose)

        for i, pose in enumerate(added_points):
            ik_pose = self.get_ik(pose)
            ik_poses.append(ik_pose)

        fk_pos1 = [forwardKinematics(q) for q in added_joints]
        fk_pos2 = [forwardKinematics(ik_pose) for ik_pose in ik_poses]

        for i, (fk_p1, fk_p2) in enumerate(zip(fk_pos1, fk_pos2)):
            calc_epsilons.append(self.norm(fk_p1, fk_p2) < epsilon)

        # Check if norm condition has been satisfied
        if all(calc_epsilons):
            return cartesian_points
        else:
            return self.taylor_interpolate_points(cartesian_points, epsilon)

    def get_time_parametrization(self, q):
        # 1. Zadatak
        # Implementiraj vremensku parametrizaciju za Ho-Cookovu metodu
        # interpolacije polinoma (jednadžba 5.24 na str 85 u knjizi
        # Osnove robotike od profesora Kovačića
        # prima: listu s pozicijama zglobova dobivenih taylorovom interpolacijom
        # Vraća: listu trajanja pojedinog segmenta putanje (duljine m-1)

        rospy.loginfo("Finished time parametrization, segment num, m-1: {}".format(len(t)))

        return t

    def createMpmatrix(self, t):
        # 2. Zadatak
        # Kako bi mogli izračunati brzine alata u zadanim
        # međutočkama krivulje, potrebno je odrediti matricu Mp
        # Matrica Mp ovisi o vremenskoj parametrizaciji je se odnosi na
        # sve segmente planirane krivulje osim prvog i zadnjeg
        # U navedenoj metodi potrebno je napraviti matricu (np.array)
        # dimenzija (m-2) X (m-4) koristeći već izračunatu vremensku parametrizaciju
        # Prima: listu trajanja pojedinog segmenta putanje
        # Vraća: Matric Mp dimenzija (m-2) x (m-4)
        # Napomena: Jednadžba 5.29 u knjizi profesora Kovačića

        m_1 = len(t)
        Mp = np.zeros((m_1 - 1, m_1 - 3))
        for i, t_ in enumerate(t):
            try:
               # TODO: Add your code here
            except Exception as e:
                pass

        rospy.loginfo("Finished adding elements to the Mp matrix, dimensions (m-2) x (m-4) : {}".format(Mp.shape))

        return Mp

    def createMmatrix(self, Mp, t):
        # 3. zadatak
        # Kako bi mogli izračunati brzine alata u svim točkama putanje (na svim segmentima)
        # potrebno je proširiti već dobivenu Mp matricu da uzima u obzir prvi i zadnji
        # segment putanje
        # prima: matricu Mp dimenzija (m-2) x (m-4), listu trajanja pojedinog segmenta putanje
        # vraća: matricu M dimenzija (m-2) x (m-2)
        # Napomena: Jednadžba 5.50 u knjizi profesora Kovačića (koristite hstack)

        m_1 = len(t)
        M1col = np.zeros((m_1 - 1, 1))
        Mlcol = np.zeros((m_1 - 1, 1))

        # TODO: Add your code here

        return M

    def createApmatrix(self, q, t):
        # 4. Zadatak
        # Kako bi mogli izračunati brzine alata u zadanim
        # međutočkama krivulje, potrebno je odrediti matricu Ap
        # Matrica Ap ovisi o vremenskoj parametrizaciji i poziciji  se odnosi na
        # zglobova u pojedinim međutočkama krivulje. Kao i Mp, uzima
        # u obzir sve segmente krivulje osim prvog i zadnjeg.
        # U navedenoj metodi potrebno je napraviti matricu (np.array)
        # dimenzija n X (m-4)
        # Prima: listu pozicija zglobova u međutočkama, listu trajanja pojedinog segmenta putanje
        # Vraća: Matrica Ap dimenzija np x (m-4)
        # Napomena: Jednadžba 5.29 u knjizi profesora Kovačića

        n = q[0].shape[0]
        m_1 = len(t)
        Ap = np.zeros((n, m_1 - 3))
        for i , t_ in enumerate(t):
            try:
                # TODO: Add your code here
            except Exception as e:
                pass

        rospy.loginfo("Finished adding elements to the Ap matrix, dimensions n x (m-4) : {}".format(Ap.shape))

        return Ap

    def createAmatrix(self, q, t, Ap):
        # 5. zadatak
        # Kako bi mogli izračunati brzine alata u svim točkama putanje (na svim segmentima)
        # potrebno je proširiti već dobivenu Ap matricu da uzima u obzir prvi i zadnji
        # segment putanje
        # prima: matricu Ap dimenzija n x (m-4), listu trajanja pojedinog segmenta putanje
        # vraća: matricu A dimenzija n x (m-2)
        # Napomena: Jednadžba 5.50 u knjizi profesora Kovačića (koristite hstack)

        # TODO: Add your code here

        return A

    def calcultate_dq(self, A, M):
        # 6. zadatak
        # Koristeći izračunate A i M matrice, izračunajte brzine
        # zglobova u svim međutočkama (5.50)
        # Nakon toga dodajte prvu i posljednju točku čije
        # su brzine zglobova jednake 0, (primjer 5.6.2)
        # prima: matricu A dimenztija n X (m-2), matricu M dimenzija (m-2) x (m-2)
        # vraća: matricu Dq dimenzija n x (m)

        # TODO: Add your code here

        return dQ


    def ho_cook(self, cartesian):
        # 6. Zadatak

        # List of joint positions that must be visited
        q =  [self.get_ik(pos) for pos in cartesian]
        print("Inverse kinematics q is: {}".format(q))
        # Time parametrization
        t = self.get_time_parametrization(q)
        print(t)
        # Ap matrix
        Ap = self.createApmatrix(q, t)
        print("Ap [{}] is: {}".format(Ap.shape, Ap))
        Mp = self.createMpmatrix(t)
        print("Mp [{}] is: {}".format(Mp.shape, Mp))
        A = self.createAmatrix(q, t, Ap)
        print("A [{}] is: {}".format(A.shape, A))
        M = self.createMmatrix(Mp, t)
        print("M [{}] is: {}".format(M.shape, M))
        dq = self.calculate_dq()
        print("dq [{}] is: {}".format(dq.shape, dq))


        Bk = []
        for k, t_ in enumerate(t):
            if k == 1:
                Bfirst = self.getBfirstSeg(q, dq, t)
            if k > 1 and k < len(t) - 1:
                Bk.append(self.getBanySeg(q, dq, t, k))
            else:
                Blast = self.getBLastSeg(q, dq, t)

        #print("Bfirst: {}".format(Bfirst))
        #print("Bk: {}".format(Bk))
        #print("Blast: {}".format(Blast))

        trajectory = self.createTrajectory(q, dq, t)
        #print(trajectory)
        self.trajectory_pub.publish(trajectory)

    def go_to_pose_ho_cook(self, goal_pose, eps):

        # Reset saved ee_points and fk points
        self.ee_points = []; self.ee_points_fk = []
        start_time = rospy.Time.now().to_sec()
        # Calculate Taylor points
        points = self.taylor_interpolate_points([poseToArray(self.current_pose), poseToArray(goal_pose)], eps)
        # Add time parametrization
        q, dq = self.ho_cook(points)

        draw(self.ee_points, self.ee_points_fk, points, eps)

    def go_to_pose_taylor(self, goal_pose, eps):
        # Reset saved ee_points and fk points
        self.ee_points = []; self.ee_points_fk = []
        # Start time counting
        start_time = rospy.Time.now().to_sec()
        # calculate taylor points
        points = self.taylor_interpolate_points([poseToArray(self.current_pose), poseToArray(goal_pose)], eps)
        # get ik_poses from calculated_taylor_points (do not remove first point anymore :))
        ik_poses = [self.get_ik(pose) for pose in points]
        duration = rospy.Time.now().to_sec() - start_time
        # Info print
        rospy.logdebug("Number of points is: {}".format(len(points)))
        rospy.logdebug("Taylor interpolation duration is: {}".format(duration))
        # Execute q_cmds
        self.execute_cmds(ik_poses)
        # draw_path
        draw(self.ee_points, self.ee_points_fk, points, eps)



    def run(self):
        # Initialize starting pose
        self.init_pose()
        # Sleep for 5. seconds
        rospy.sleep(10.0)
        # Initialize goal pose
        start_pose = createGoalPose(0.2, 0.25, 1.6, -0.5, 0.5, 0.5, 0.5)
        wanted_pose1 = createGoalPose(0.2, 0.25, 1.4, -0.5, 0.5, 0.5, 0.5)
        wanted_pose2 = createGoalPose(0.3, 0.25, 1.4, -0.5, 0.5, 0.5, 0.5)
        wanted_pose3 = createGoalPose(0.2, 0.25, 1.6, -0.5, 0.5, 0.5, 0.5)

        self.change_to_pos_client.call()
        while not rospy.is_shutdown():
            test = True
            if test:

                self.go_to_start_pose(start_pose)
                rospy.sleep(3.0)

                self.change_to_traj_client.call()
                eps = 0.002
                self.go_to_pose_ho_cook(wanted_pose1, eps)
                rospy.sleep(3.0)

                eps = 0.002
                self.go_to_pose_ho_cook(wanted_pose2, eps)
                rospy.sleep(3.0)

                eps = 0.002
                self.go_to_pose_ho_cook(wanted_pose3, eps)

                #eps = 0.05
                #self.go_to_pose_ho_cook(wanted_pose2, eps)

                break

        exit()


if __name__ == "__main__":
    lab3 = OrLab3()
    lab3.run()