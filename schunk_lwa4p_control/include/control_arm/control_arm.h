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
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>
#include <std_srvs/Trigger.h>
#include <controller_manager_msgs/SwitchController.h>
#include <controller_manager_msgs/ListControllers.h>

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

// Conversions 
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

// Utils
#include <tf2/LinearMath/Quaternion.h>



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


    private: 
        
        // Reads and verifies ROS parameters 
        bool readParameters(); 

        // Initialization method
        void init();     

        // ROS node handle  
        ros::NodeHandle nodeHandle_;
        ros::NodeHandle nodeHandleWithoutNs_;

        // ROS Publishers and subscribers
        ros::Publisher displayTrajectoryPublisher_;
        ros::Publisher currentPosePublisher_;
        ros::Publisher cmdJoint1Publisher;
        ros::Publisher cmdJoint2Publisher;
        ros::Publisher cmdJoint3Publisher;
        ros::Publisher cmdJoint4Publisher;
        ros::Publisher cmdJoint5Publisher;
        ros::Publisher cmdJoint6Publisher;
        ros::Publisher cmdJointGroupPublisher;

        ros::Subscriber armCmdPoseSubscriber_;
        ros::Subscriber armCmdDeltaPoseSubscriber_; 
        ros::Subscriber armCmdToolOrientationSubscriber_;

        // ROS Services
        ros::ServiceServer disableCollisionService_;
        ros::ServiceServer addCollisionObjectService_;
        ros::ServiceServer startPositionControllersService_;
        ros::ServiceServer startJointTrajectoryControllerService_;
        ros::ServiceServer startJointGroupPositionControllerService_;
        ros::ServiceServer sendArmToHomingPoseService_;

        // ROS Service clients
        ros::ServiceClient applyPlanningSceneServiceClient_;
        ros::ServiceClient realRobotDriverInitServiceClient_;
        ros::ServiceClient addCollisionObjectServiceClient_;
        ros::ServiceClient switchControllerServiceClient_;
        ros::ServiceClient listControllersServiceClient_;
        ros::ServiceClient switchToPositionControllerServiceClient_;
        ros::ServiceClient switchToTrajectoryControllerServiceClient_;

        // ROS Subscriber Callback
        void cmdPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);
        void cmdDeltaPoseCallback(const geometry_msgs::Pose::ConstPtr& msg); 
        void cmdToolOrientationCallback(const geometry_msgs::Point::ConstPtr& msg); 

        // ROS Services callbacks
        bool disableCollisionServiceCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
        bool addCollisionObjectServiceCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
        bool startPositionControllers(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
        bool startJointTrajectoryController(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
        bool startJointGroupPositionController(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
        bool sendArmToHomingPose(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

        // DisplayTrajectory
        moveit_msgs::DisplayTrajectory displayTrajectory_;         

        // Private variables
        int sleepMs_;
        bool realRobot_;
        bool enableVisualization_;
        bool moveGroupInitialized_;   
        bool planningSceneInitialized_; 
        bool firstTrajectoryExecution_ = true;
        bool blockingMovement = false; 
        geometry_msgs::Pose m_cmdPose;    
        geometry_msgs::Pose m_cmdDeltaPose;

        // Vectors and arrays
        std::vector<double> m_jointPositions_;

        float round(float var);
        bool sendZeros(std::string ControllerType);
        bool sendToCmdPose(); 
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

       
};



#endif  // CONTROL_ARM_H