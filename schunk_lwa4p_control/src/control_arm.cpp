#include "control_arm/control_arm.h"


ControlArm::ControlArm(ros::NodeHandle nh) : nodeHandle_(nh)
{



        ROS_INFO("[ControlArm] Node started."); 

        // Read parameters from config file. 
        if (!readParameters()) {
            ros::requestShutdown(); 
        }

        // Initial sleep (waiting for move group and rest of the MoveIt stuff to initialize.)
        usleep(sleepMs_);

        // Initialize class 
        init(); 

        // Find out basic info
        getBasicInfo(); 

}
    
ControlArm::~ControlArm() {

}

bool ControlArm::readParameters() {
    // Load common parameters
    nodeHandle_.param("visualization/enable_viz", enableVisualization_, true);     
    nodeHandle_.param("init/sleep_time", sleepMs_, 20000000); 

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
    std::string armCmdSendToPoseTopicName; 
    int armCmdSendToPoseTopicQueueSize;

    nodeHandle_.param("publishers/display_trajectory_topic", displayTrajectoryTopicName, std::string("move_group/display_planned_path")); 
    nodeHandle_.param("publishers/queue_size", displayTrajectoryQueueSize, 1); 
    nodeHandle_.param("subscribers/cmd_pose_topic", cmdPoseTopicName, std::string("arm/command/pose"));
    nodeHandle_.param("subscribers/queue_size", cmdPoseTopicQueueSize, 1); 

    ROS_INFO("[ControlArm] Initializing subscribers/publishers." );


    displayTrajectoryPublisher_ = nodeHandle_.advertise<moveit_msgs::DisplayTrajectory>(displayTrajectoryTopicName, displayTrajectoryQueueSize);
    armCmdPoseSubscriber_ = nodeHandle_.subscribe<geometry_msgs::Pose>(cmdPoseTopicName, cmdPoseTopicQueueSize, &ControlArm::cmdPoseCallback, this);
    //armCmdSendToPoseSubscriber_ = nodeHandle_.subscribe<std_msgs::bool>(armCmdSendToPoseTopicName, armCmdSendToPoseTopicQueueSize, &ControlArm::cmdSendToPoseCallback, this); 
    
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

        ROS_INFO("[ControlArm] Reference planning frame: %s", m_moveGroupPtr->getPlanningFrame().c_str());

        ROS_INFO("[ControlArm] Reference end effector frame: %s", m_moveGroupPtr->getEndEffectorLink().c_str());

    }

}

void ControlArm::getCurrentEEPose() {
    
}

void ControlArm::cmdPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {

    ROS_INFO("[ControlArm] Recieved cmd_pose...");
    
    // Set CMD pose
    m_cmdPose.position = msg->position;
    m_cmdPose.orientation = msg->orientation; 

    sendToCmdPose(); 

}


// Would have more sense to create it as service 
//void ControlArm::cmdSendToPoseCallback(const std_msgs::Bool::ConstPtr& msg) {
    //    moveReady_ = msg->data; 
//}

bool ControlArm::sendToCmdPose(){

    setCmdPose(); 

    // Call planner, compute plan and visualize it

    moveit::planning_interface::MoveGroupInterface::Plan plannedPath; 

    // plan Path   
    bool success = (m_moveGroupPtr->plan(plannedPath) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


    ROS_INFO("[ControlArm] Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
    
    // Requires async spinner to
    if (success){
        m_moveGroupPtr->move(); 
    }

    return success; 
}

void ControlArm::run() {

    ros::Rate r(50);

    while(ros::ok)
    {
        ROS_INFO("[ControlArm] Running...");
        r.sleep(); 
    }

}




