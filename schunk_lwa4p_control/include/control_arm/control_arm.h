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

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/DisplayTrajectory.h>



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
        moveit::planning_interface::MoveGroupInterface *m_moveGroupPtr;  

        // Run method
        void run(); 


    private: 
        
        // Reads and verifies ROS parameters 
        bool readParameters(); 

        // Initialization method
        void init();     

        // ROS node handle  
        ros::NodeHandle nodeHandle_;         

        // ROS Publishers and subscribers
        ros::Publisher displayTrajectoryPublisher_; 
        ros::Subscriber armCmdPoseSubscriber_;

        // ROS Subscriber Callback
        void cmdPoseCallback(const geometry_msgs::Pose::ConstPtr& msg);

        // DisplayTrajectory
        moveit_msgs::DisplayTrajectory displayTrajectory_; 

        // Private variables
        int sleepMs_;
        bool isNodeRunning_; 
        bool enableVisualization_;      
        bool moveGroupInitialized_;   
        bool planningSceneInitialized_; 
        bool moveReady_;
        geometry_msgs::Pose m_cmdPose;        
        
        // Private methods  
        bool sendToCmdPose(); 
        bool sendToZeroPose();
        bool planPathToCmdPose();
        bool planPathToZeroPose(); 
        bool isNodeRunning(); 



       
};



#endif  // CONTROL_ARM_H