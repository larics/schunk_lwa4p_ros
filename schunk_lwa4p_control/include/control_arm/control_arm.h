#ifndef CONTROL_ARM_H 
#define CONTROL_ARM_H

#include <pthread.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <vector>
#include <thread>
#include <unistd.h>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>
#include <std_srvs/Trigger.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/PositionIKRequest.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionIKRequest.h>
#include <moveit_msgs/GetPositionIKResponse.h>

// Conversions 
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// Utils
#include <tf2/LinearMath/Quaternion.h>

#include "schunk_lwa4p_control/CartesianPath.h"
#include "wsg_50_common/Move.h"
#include "wsg_50_common/Conf.h"
#include "christmas_fair_common/StartTrajectorySrv.h"



class ControlArm{

    public:

        // Constructor
        ControlArm(ros::NodeHandle nh); 

        // Destructor 
        ~ControlArm(); 

        // Variables
        geometry_msgs::Point currentEEPosition; 
        geometry_msgs::Pose currentEEPose; 
        
        // Setters 
        bool setCmdPose();
        bool setCmdJoint();
        bool setMoveGroup(); 
        bool setPlanningScene();

        // Getters
        void getBasicInfo(); 
        bool getCmdPose(); 
        geometry_msgs::Point getCurrentEEPosition(); 
        geometry_msgs::Pose getCurrentEEPose(); 

        // Pointer to move group (https://answers.ros.org/question/344598/cant-create-movegroupinterface-object-in-my-own-class/)
        moveit::planning_interface::MoveGroupInterface      *m_moveGroupPtr;
        const robot_state::JointModelGroup                  *m_jointModelGroupPtr;
        planning_scene::PlanningScene                       *m_planningScenePtr;
        moveit::core::RobotStatePtr                         m_currentRobotStatePtr;
        robot_model_loader::RobotModelLoader                m_robotModelLoader;
        Eigen::Affine3d                                     m_endEffectorState;

        // Run method
        void run(); 

    // It's not possible to access private members of this class from another clas
    // Link to issue that explains it: https://stackoverflow.com/questions/18944451/how-to-make-a-derived-class-access-the-private-member-data
    private: 
        
        // Reads and verifies ROS parameters 
        bool readParameters(); 

        // Initialization method
        void init();     

        // ROS node handle  
        ros::NodeHandle nodeHandle_;
        ros::NodeHandle nodeHandleWithoutNs_;

        // transformListener
        tf::TransformListener listener;
        tf::TransformBroadcaster broadcaster;

        // ROS Publishers
        ros::Publisher displayTrajectoryPublisher_;
        ros::Publisher currentPosePublisher_;
        ros::Publisher cmdJoint1Publisher;
        ros::Publisher cmdJoint2Publisher;
        ros::Publisher cmdJoint3Publisher;
        ros::Publisher cmdJoint4Publisher;
        ros::Publisher cmdJoint5Publisher;
        ros::Publisher cmdJoint6Publisher;
        ros::Publisher cmdJointGroupPositionPublisher;
        ros::Publisher cmdJointGroupVelocityPublisher;
        ros::Publisher powerline0PosePublisher;
        ros::Publisher powerline1PosePublisher;

        // CHRISMAS Action schunks
        ros::Publisher schunkAction1Publisher;
        ros::Publisher schunkAction2Publisher;
        ros::Publisher schunkAction3Publisher;
        ros::Publisher schunkAction4Publisher;

        // ROS Subscribers
        ros::Subscriber armCmdPoseSubscriber_;
        ros::Subscriber armCmdDeltaPoseSubscriber_; 
        ros::Subscriber armCmdToolOrientationSubscriber_;

        // ROS Services
        ros::ServiceServer disableCollisionService_;
        ros::ServiceServer addCollisionObjectService_;
        ros::ServiceServer startPositionControllersService_;
        ros::ServiceServer startJointTrajectoryControllerService_;
        ros::ServiceServer startJointGroupPositionControllerService_;
        ros::ServiceServer startJointGroupVelocityControllerService_;
        ros::ServiceServer sendArmToHomingPoseService_;
        ros::ServiceServer checkIKSolutionsService_;
        ros::ServiceServer executeCartesianPathService_;

        // CHRISTMAS Services
        ros::ServiceServer pickSugarService_;
        ros::ServiceServer putSugarService_;
        ros::ServiceServer returnSugarService_;
        ros::ServiceServer homingService_;

        // ROS Service clients
        ros::ServiceClient applyPlanningSceneServiceClient_;
        ros::ServiceClient realRobotDriverInitServiceClient_;
        ros::ServiceClient addCollisionObjectServiceClient_;
        ros::ServiceClient switchControllerServiceClient_;
        ros::ServiceClient listControllersServiceClient_;
        ros::ServiceClient switchToPositionControllerServiceClient_;
        ros::ServiceClient getIKSolutionsServiceClient_;
        ros::ServiceClient switchToTrajectoryControllerServiceClient_;

        // Only gripper clients! --> Move to action server
        ros::ServiceClient gripperGraspServiceClient_;
        ros::ServiceClient gripperMoveServiceClient_;
        ros::ServiceClient gripperSetForceServiceClient_;
        ros::ServiceClient gripperReleaseServiceClient_;

        // ROS Subscriber Callback
        void cmdPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
        void cmdDeltaPoseCallback(const geometry_msgs::Pose::ConstPtr& msg); 
        void cmdToolOrientationCallback(const geometry_msgs::Point::ConstPtr& msg);

        // CHRISTMAS service callbacks
        bool schunkPickSugarServiceCallback(christmas_fair_common::StartTrajectorySrvRequest &req, christmas_fair_common::StartTrajectorySrvResponse &res);
        bool schunkPutSugarServiceCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
        bool schunkReturnSugarServiceCallback(christmas_fair_common::StartTrajectorySrvRequest &req, christmas_fair_common::StartTrajectorySrvResponse &res);
        bool schunkHomingServiceCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);


        // ROS Services callbacks
        bool disableCollisionServiceCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
        bool addCollisionObjectServiceCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
        bool startPositionControllers(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
        bool startJointTrajectoryController(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
        bool startJointGroupPositionController(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
        bool startJointGroupVelocityController(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
        //https://ros-planning.github.io/moveit_tutorials/doc/trac_ik/trac_ik_tutorial.html
        bool checkIKSolutionsServiceCallback(moveit_msgs::GetPositionIKRequest &req, moveit_msgs::GetPositionIKResponse &res);
        bool executeCartesianServiceCallback(schunk_lwa4p_control::CartesianPathRequest &req, schunk_lwa4p_control::CartesianPathResponse &res);
        bool sendArmToHomingPose(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

        // DisplayTrajectory
        moveit_msgs::DisplayTrajectory displayTrajectory_;

        // Private variables
        int sleepMs_;
        bool startChristmas_;
        bool enableVisualization_;
        bool moveGroupInitialized_;
        bool planningSceneInitialized_;
        bool firstTrajectoryExecution_ = true;
        bool blockingMovement = true;

        int m_schunkPickId = 0;
        int m_returnPickId = 0;

        // CHRISTMAS booleans
        bool m_startSchunkPick = false;
        bool m_putSchunkSugar = false;
        bool m_returnSchunkSugar = false;
        bool m_homingSchunk = false;

        geometry_msgs::Pose m_cmdPose;
        geometry_msgs::Pose m_cmdDeltaPose;
        sensor_msgs::JointState m_cmdJoint;


    // Vectors and arrays
        std::vector<double> m_jointPositions_;

        bool sendZeros(std::string ControllerType);
        bool planToCmdPose();
        bool executeMovement();
        bool sendToCmdPose();
        bool sendToCmdJoint();
        void sendToCmdPoses(std::vector<geometry_msgs::Pose> poses);
        void sendToJointCmdPoses(std::vector<sensor_msgs::JointState> joint_poses);
        bool sendToDeltaCmdPose();
        bool sendToZeroPose();
        bool planPathToCmdPose();
        bool planPathToZeroPose();
        bool isNodeRunning();
        bool executeDummyCartesianPath();
        void addCollisionObject(moveit_msgs::PlanningScene& planningScene);
        void getCurrentArmState();
        void getCurrentEndEffectorState(const std::string linkName);
        void getJointPositions(const std::vector<std::string>& jointNames, std::vector<double> &jointGroupPositions) ;
        void getRunningControllers(std::vector<std::string> &runningControllerNames);
        bool getIK(const std::size_t attempts, double timeout);
        Eigen::MatrixXd getJacobian(Eigen::Vector3d refPointPosition);          // Can be created as void and arg passed to be changed during execution
        Eigen::MatrixXd getInertiaMatrix(Eigen::Vector3d refPointPosition);

        // TODO: Move this to utils.cpp
        float round(float var);
        double VectorSize(geometry_msgs::Vector3 vector);
        double DotProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B);
        geometry_msgs::Vector3 getClosestPointOnLine(geometry_msgs::Vector3 line_point, geometry_msgs::Vector3 line_vector, geometry_msgs::Vector3 point);



};



#endif  // CONTROL_ARM_H