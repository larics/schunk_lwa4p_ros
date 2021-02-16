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
    std::string cmdToolOrientationTopicName; 
    int cmdToolOrientationTopicQueueSize; 

    nodeHandle_.param("publishers/display_trajectory_topic", displayTrajectoryTopicName, std::string("move_group/display_planned_path")); 
    nodeHandle_.param("publishers/queue_size", displayTrajectoryQueueSize, 1); 
    nodeHandle_.param("subscribers/cmd_pose_topic", cmdPoseTopicName, std::string("arm/command/pose"));
    nodeHandle_.param("subscribers/queue_size", cmdPoseTopicQueueSize, 1); 
    nodeHandle_.param("subscribers/cmd_tool_orientation_topic",  cmdToolOrientationTopicName, std::string("tool/command/orientation")); 
    nodeHandle_.param("subscribers/queue_size", cmdToolOrientationTopicQueueSize, 1);

    ROS_INFO("[ControlArm] Initializing subscribers/publishers." );

    displayTrajectoryPublisher_ = nodeHandle_.advertise<moveit_msgs::DisplayTrajectory>(displayTrajectoryTopicName, displayTrajectoryQueueSize);
    armCmdPoseSubscriber_ = nodeHandle_.subscribe<geometry_msgs::Pose>(cmdPoseTopicName, cmdPoseTopicQueueSize, &ControlArm::cmdPoseCallback, this);
    armCmdToolOrientationSubscriber_ = nodeHandle_.subscribe<geometry_msgs::Point>(cmdToolOrientationTopicName, cmdToolOrientationTopicQueueSize, &ControlArm::cmdToolOrientationCallback, this); 
    //armCmdSendToPoseSubscriber_ = nodeHandle_.subscribe<std_msgs::bool>(armCmdSendToPoseTopicName, armCmdSendToPoseTopicQueueSize, &ControlArm::cmdSendToPoseCallback, this); 
    
}

bool ControlArm::setMoveGroup() {

    ROS_INFO("[ControlArm] Setting move group." );

    // MoveIt move group
    static const std::string groupName = "arm"; 
    m_moveGroupPtr = new moveit::planning_interface::MoveGroupInterface(groupName); 

    // Get current robot arm state
    getCurrentArmState(); 

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

void ControlArm::cmdPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {

    ROS_INFO("[ControlArm] Recieved cmd_pose...");
    
    // Set CMD pose
    m_cmdPose.position = msg->position;
    m_cmdPose.orientation = msg->orientation; 

    sendToCmdPose(); 

}

void ControlArm::cmdToolOrientationCallback(const geometry_msgs::Point::ConstPtr& msg) {

    ROS_INFO("[ControlArm] Received cmd tool orientation..."); 

    // Get current end effector state 
    getCurrentEndEffectorState("lwa4p_link6"); 

    geometry_msgs::Pose cmdPose; 
    tf2::Quaternion cmdQuaternion; 

    // Set current end effector position as command
    cmdPose.position.x = m_endEffectorState(0, 3);
    cmdPose.position.y = m_endEffectorState(1, 3);
    cmdPose.position.z = m_endEffectorState(2, 3);

    // Set commanded roll, pitch, yaw as quaternion 
    cmdQuaternion.setRPY(msg->x, msg->y, msg->z); 
    cmdPose.orientation.x = cmdQuaternion.x(); 
    cmdPose.orientation.y = cmdQuaternion.y(); 
    cmdPose.orientation.z = cmdQuaternion.z(); 
    cmdPose.orientation.w = cmdQuaternion.w(); 

    // set CMD pose
    m_cmdPose = cmdPose; 

    sendToCmdPose();  

}

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


void ControlArm::getCurrentArmState() {

    // method is more like refresh current kinematic state (getCurrentKinematicState)
    m_currentRobotStatePtr = m_moveGroupPtr->getCurrentState(); 

}

void ControlArm::getCurrentEndEffectorState(const std::string endEffectorLinkName) {

    m_endEffectorState = m_currentRobotStatePtr->getGlobalLinkTransform(endEffectorLinkName); 

    bool debug = true; 
    if (debug){

        ROS_INFO_STREAM("Translation: \n" << m_endEffectorState.translation() << "\n"); 
        ROS_INFO_STREAM("Rotation: \n" << m_endEffectorState.rotation() << "\n"); 

    }

}

void ControlArm::getJointPositions(const std::vector<std::string>& jointNames) {
    std::vector<double> jointGroupPositions; 
    m_currentRobotStatePtr->copyJointGroupPositions(m_jointModelGroupPtr, jointGroupPositions);

    bool debug = false; 
    if (debug){
        for (std::size_t i = 0; i < jointNames.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", jointNames[i].c_str(), jointGroupPositions[i]);
        }

    }
  
}

bool ControlArm::getIK(const std::size_t attempts, double timeout) {

    bool found_ik = m_currentRobotStatePtr->setFromIK(m_jointModelGroupPtr, m_endEffectorState, attempts, timeout);

    bool debug = true; 
    if (debug){
        ROS_INFO("Found IK solution!"); 

    }

    return found_ik; 

}

Eigen::MatrixXd ControlArm::getJacobian(Eigen::Vector3d refPointPosition){

    Eigen::MatrixXd jacobianMatrix; 
    m_currentRobotStatePtr->getJacobian(m_jointModelGroupPtr, 
                                        m_currentRobotStatePtr->getLinkModel(m_jointModelGroupPtr->getLinkModelNames().back()),
                                        refPointPosition, 
                                        jacobianMatrix);

    return jacobianMatrix; 

}

void ControlArm::run() {

    ros::Rate r(50);

    while(ros::ok)
    {
        // ROS_INFO("[ControlArm] Running...");
        // getCurrentArmState(); 
        // ROS_INFO("[ControlArm] Current arm state is ", *m_currentRobotStatePtr);
        // ROS_INFO("Running!");

        // Get current joint position for every joint in robot arm 
        getCurrentArmState(); 

        // Get all joints 
        m_jointModelGroupPtr = m_currentRobotStatePtr->getJointModelGroup("arm"); 

        // Get current joint positions 
        getJointPositions(m_jointModelGroupPtr->getVariableNames());  

        // Call get current end effector state to setup pointer of variable which is used in get Inverse Kinematics
        // getCurrentEndEffectorState("lwa4p_link6"); 

        // Call to get IK 
        //std::size_t attempts = 10; 
        //double timeout = 1; 
        //bool successIK; 
        //successIK = getIK(attempts, timeout); 

        //Eigen::MatrixXd m_; 
        //Eigen::Vector3d testVector(0.0, 0.0, 0.0);
        //m_ = getJacobian(testVector);
        //ROS_INFO_STREAM("Jacobian: \n" << m_); 
    
        r.sleep(); 
    }

}




