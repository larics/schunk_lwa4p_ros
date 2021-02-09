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

// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

// MoveIt
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/DisplayTrajectory.h>



class ControlArm{

    public:

        // Constructor
        explicit ControlArm(ros::NodeHandle *nh); 

        // Destructor 
        ~ControlArm(); 

        // Everything under is possibly redundant 

        // Variables
        geometry_msgs::Point currentEEPosition; 
        geometry_msgs::Pose currentEEPose; 
        
        // Setters 
        bool setCmdPose();
        bool setPlanningGroup(); 
        bool setMoveGroup(); 

        // Getters
        void getBasicInfo(); 
        bool getCmdPose(); 
        std::string getMoveGroupName();
        geometry_msgs::Point getCurrentEEPosition(); 
        geometry_msgs::Pose getCurrentEEPose(); 

        /


    private: 
        
        // Reads and verifies ROS parameters 
        bool readParameters(); 

        void init();      

        // ROS node handle  
        ros::NodeHandle nodeHandle_; 

        // Control arm on thread
        std::thread controlArmThread_; 

        // MoveIt objects!
        moveit::planning_interface::MoveGroupInterface armMoveGroup_; 
        moveit::planning_interface::PlanningSceneInterface planningScene_; 

        moveit_msgs::DisplayTrajectory displayTrajectory_; 

        // Private variables
        std::string m_planningGroup;  
        geometry_msgs::Pose m_cmdPose; 

        

        // Private methods  
        bool sendToCmdPose(); 
        bool sendToZeroPose();
        bool planPathToCmdPose();
        bool planPathToZeroPose(); 
        bool isNodeRunning(); 

       
};



#endif  // CONTROL_ARM_H