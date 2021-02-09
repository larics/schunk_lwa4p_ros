#include "control_arm/control_arm.h"


ControlArm::ControlArm(ros::NodeHandle nh) : nodeHandle_(nh)
{

        ROS_INFO("[ControlArm] Node started."); 

        // Read parameters from config file. 
        if (!readParameters()) {
            ros::requestShutdown(); 
        }

        init(); 

}
    
ControlArm::~ControlArm() {

}

bool ControlArm::readParameters() {
    // Load common parameters
    nodeHandle_.param("visualization/enable_viz", enableVisualization_, true);     

    return true; 
}

void ControlArm::init() {

    ROS_INFO("[ControlArm] Started node initialization.");


    // Set main move group
    moveGroupInitialized_ = setMoveGroup(); 

    planningSceneInitialized_ = setPlanningScene(); 



    // Initialize publishers and subscribers;
    std::string displayTrajectoryTopicName; 
    int displayTrajectoryQueueSize; 
    std::string cmdPoseTopicName; 
    int cmdPoseTopicQueueSize; 

    nodeHandle_.param("publishers/display_trajectory_topic", displayTrajectoryTopicName, std::string("move_group/display_planned_path")); 
    nodeHandle_.param("publishers/queue_size", displayTrajectoryQueueSize, 1); 
    nodeHandle_.param("subscribers/cmd_pose_topic", cmdPoseTopicName, std::string("arm/command/pose"));
    nodeHandle_.param("subscribers/queue_size", cmdPoseTopicQueueSize, 1); 
    
}

bool ControlArm::setMoveGroup() {

    ROS_INFO("[ControlArm] Setting move group." );

    // MoveIt move group
    static const std::string groupName = "arm"; 
    m_moveGroupPtr = new moveit::planning_interface::MoveGroupInterface(groupName); 

    return true;

}


bool ControlArm::setPlanningScene() {

    ROS_INFO("[ControlArm] Setting planning scene."); 


    // MoveIt planning scene
    moveit::planning_interface::PlanningSceneInterface m_planningScene; 

    return true; 
}

bool ControlArm::setCmdPose() {

    if (moveGroupInitialized_) {
        m_moveGroupPtr->setPoseTarget(m_cmdPose);
    }

}

void ControlArm::getBasicInfo() {

    if (moveGroupInitialized_) {

        ROS_INFO("[ControlArm] Reference frame: %s", m_moveGroupPtr->getPlanningFrame().c_str());

        ROS_INFO("[ControlArm] Reference frame: %s", m_moveGroupPtr->getEndEffectorLink().c_str());

    }

}

void ControlArm::run() {

}




