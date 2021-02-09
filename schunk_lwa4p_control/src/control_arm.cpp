#include "control_arm/control_arm.h"


ControlArm::ControlArm(ros::NodeHandle nh) 
    : nodeHandle_(nh) {

        ROS_INFO("[ControlArm] Node started."); 

        // Read parameters from config file. 
        if (!readParameters()) {
            ros::requestShutdown(); 
        }

        init(); 

    }



}

ControlArm::~ControlArm() {
    {
        boost::unique_lock<boost::shared_mutex> lockNodeStatus(mutexNodeStatus_);
        isNodeRunning_ = false;
    }

    controlThread_.join()
}

bool readParameters() {
    // Load common parameters
    nodeHandle_.param("visualization/enable_viz", enableVisualization_, true);     

    return true; 
}

void ControlArm::init() {

    ROS_INFO("[ControlArm] Started node initialization".)

    // Set main move group
    setMoveGroup(); 

    // Initialize publisher and subscriber;
    std::string displayTrajectoryTopicName; 
    int displayTrajectoryQueueSize; 

    nodeHandle_.param("publishers/display_trajectory_topic", displayTrajectoryTopicName, std::string("move_group/display_planned_path")); 
    nodeHandle_.param("publishers/queue_size", displayTrajectoryQueueSize, 1); 
    nodeHandle_.param("subscribers/cmd_pose_topic", cmdPoseTopicName, std::string("arm/command/pose"));
    nodeHandle_.param("subscribers/queue_size", cmdPoseTopicQueueSize, 1); 

    
}

void ControlArm::setMoveGroup() {

    ROS_INFO("[ControlArm] Setting move group." )

    armMoveGroup(m_planningGroup); 

}

bool ControlArm::setCmdPose() {

    m_planningGroup.setPoseTarget(m_cmdPose); 

}

void ControlArm::getBasicInfo() {

    ROS_INFO("[ControlArm] Reference frame: %s", armMoveGroup_.getPlanningFrame().c_str());

    ROS_INFO("[ControlArm] Reference frame: %s", armMoveGroup_.getEndEffectorLink().c_str());
}



