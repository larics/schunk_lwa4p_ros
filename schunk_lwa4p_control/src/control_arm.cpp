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
    std::string currentPoseTopicName;
    int currentPoseTopicQueueSize;
    std::string cmdPoseTopicName; 
    int cmdPoseTopicQueueSize; 
    std::string cmdToolOrientationTopicName; 
    int cmdToolOrientationTopicQueueSize; 
    std::string cmdDeltaPoseTopicName; 
    int cmdDeltaPoseTopicQueueSize;

    std::string disableCollisionServiceName;

    nodeHandle_.param("publishers/display_trajectory_topic", displayTrajectoryTopicName, std::string("move_group/display_planned_path")); 
    nodeHandle_.param("publishers/queue_size", displayTrajectoryQueueSize, 1);
    nodeHandle_.param("publishers/current_pose", currentPoseTopicName, std::string("tool/current_pose"));
    nodeHandle_.param("publishers/queue_size", currentPoseTopicQueueSize, 1);
    nodeHandle_.param("subscribers/cmd_pose_topic", cmdPoseTopicName, std::string("arm/command/pose"));
    nodeHandle_.param("subscribers/queue_size", cmdPoseTopicQueueSize, 1); 
    nodeHandle_.param("subscribers/cmd_tool_orientation_topic",  cmdToolOrientationTopicName, std::string("tool/command/orientation")); 
    nodeHandle_.param("subscribers/queue_size", cmdToolOrientationTopicQueueSize, 1);
    nodeHandle_.param("subscribers/cmd_delta_pose_topic", cmdDeltaPoseTopicName, std::string("arm/command/delta_pose")); 
    nodeHandle_.param("subscribers/queue_size", cmdDeltaPoseTopicQueueSize, 1);
    nodeHandle_.param("client/disable_collision_service", disableCollisionServiceName, std::string("tool/disable_collision"));

    ROS_INFO("[ControlArm] Initializing subscribers/publishers." );

    displayTrajectoryPublisher_ = nodeHandle_.advertise<moveit_msgs::DisplayTrajectory>(displayTrajectoryTopicName, displayTrajectoryQueueSize);
    currentPosePublisher_ = nodeHandle_.advertise<geometry_msgs::Pose>(currentPoseTopicName, currentPoseTopicQueueSize);
    armCmdPoseSubscriber_ = nodeHandle_.subscribe<geometry_msgs::Pose>(cmdPoseTopicName, cmdPoseTopicQueueSize, &ControlArm::cmdPoseCallback, this);
    armCmdToolOrientationSubscriber_ = nodeHandle_.subscribe<geometry_msgs::Point>(cmdToolOrientationTopicName, cmdToolOrientationTopicQueueSize, &ControlArm::cmdToolOrientationCallback, this); 
    armCmdDeltaPoseSubscriber_ = nodeHandle_.subscribe<geometry_msgs::Pose>(cmdDeltaPoseTopicName, cmdDeltaPoseTopicQueueSize, &ControlArm::cmdDeltaPoseCallback, this); 

    // Client for disable collisions service
    disableCollisionService_ = nodeHandle_.advertiseService(disableCollisionServiceName, &ControlArm::disableCollisionServiceCallback, this);
}

bool ControlArm::setMoveGroup() {

    ROS_INFO("[ControlArm] Setting move group." );

    // MoveIt move group
    static const std::string groupName = "arm"; 
    m_moveGroupPtr = new moveit::planning_interface::MoveGroupInterface(groupName); 

    // Allow replanning 
    m_moveGroupPtr->allowReplanning(true); 

    // Get current robot arm state
    getCurrentArmState(); 

    return true;

}

bool ControlArm::setPlanningScene() {

    ROS_INFO("[ControlArm] Setting planning scene.");

    // MoveIt planning scene setup as seen (http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/planning_scene/planning_scene_tutorial.html)
    robot_model_loader::RobotModelLoader m_robotLoader("robot_description");
    robot_model::RobotModelPtr kinematic_model = m_robotLoader.getModel();
    m_planningScenePtr = new planning_scene::PlanningScene(kinematic_model);

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

void ControlArm::cmdDeltaPoseCallback(const geometry_msgs::Pose::ConstPtr& msg) {

    ROS_INFO("[ControlArm] Recieved cmd_delta_pose...");

    // Set delta CMD pose
    m_cmdDeltaPose.position = msg->position; 
    m_cmdDeltaPose.orientation = msg->orientation; 

    sendToDeltaCmdPose(); 

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
    
    // Requires async spinner to work (Added asyncMove/non-blocking)
    if (success){
        if (blockingMovement){
            m_moveGroupPtr->move(); 
        }else {
            m_moveGroupPtr->asyncMove(); 
        }
        
    }


    return success; 
}

bool ControlArm::sendToDeltaCmdPose() {

    // populate m_cmd pose
    Eigen::Affine3d currentPose_ = m_moveGroupPtr->getCurrentState()->getFrameTransform("lwa4p_link6"); // Currently lwa4p_link6, possible to use end effector link 
    geometry_msgs::Pose currentROSPose_; 
    tf::poseEigenToMsg(currentPose_, currentROSPose_);

    ROS_INFO_STREAM("[ControlArm] currentROSPose_:" << currentROSPose_);

    geometry_msgs::Pose cmdPose; 
    cmdPose.position.x = currentROSPose_.position.x + m_cmdDeltaPose.position.x;
    cmdPose.position.y = currentROSPose_.position.y + m_cmdDeltaPose.position.y;   
    cmdPose.position.z = currentROSPose_.position.z + m_cmdDeltaPose.position.z;
    cmdPose.orientation.x = currentROSPose_.orientation.x + m_cmdDeltaPose.orientation.x;
    cmdPose.orientation.y = currentROSPose_.orientation.y + m_cmdDeltaPose.orientation.y;    
    cmdPose.orientation.z = currentROSPose_.orientation.z + m_cmdDeltaPose.orientation.z;    
    cmdPose.orientation.w = currentROSPose_.orientation.w + m_cmdDeltaPose.orientation.w;   

    ROS_INFO_STREAM("[ControlArm] currentPose: " << cmdPose);  


    // set CMD pose
    m_cmdPose = cmdPose; 

    sendToCmdPose();      

}

bool ControlArm::disableCollisionServiceCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res){

    if (planningSceneInitialized_){
        bool debugOut = true;
        collision_detection::AllowedCollisionMatrix acm = m_planningScenePtr->getAllowedCollisionMatrix();
        acm.print(std::cout);
        return true;
    }
    else{
        return false;
    }

}

float ControlArm::round(float var){

    float value = (int)(var * 1000 + .5); 
    return (float) value/1000; 

}

void ControlArm::getCurrentArmState() {

    // method is more like refresh current kinematic state (getCurrentKinematicState)
    m_currentRobotStatePtr = m_moveGroupPtr->getCurrentState(); 

}

void ControlArm::getCurrentEndEffectorState(const std::string endEffectorLinkName) {

    m_endEffectorState = m_currentRobotStatePtr->getGlobalLinkTransform(endEffectorLinkName); 

    bool debug = false; 
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

bool ControlArm::executeDummyCartesianPath(){
    geometry_msgs::Pose startPose = m_moveGroupPtr->getCurrentPose().pose; 
    
    std::vector<geometry_msgs::Pose> waypoints;
    startPose.position.z -= 0.45; 
    waypoints.push_back(startPose);

    startPose.position.x += 0.35; 
    waypoints.push_back(startPose); 

    startPose.position.z += 0.35; 
    waypoints.push_back(startPose); 

    // set moveGroup scaling factor
    m_moveGroupPtr->setMaxVelocityScalingFactor(0.10);  

    moveit_msgs::RobotTrajectory trajectory; 
    double eefStep = 0.01; double jumpTreshold = 0.00; // in real-world applications this jump Threshold must be > 0; 
    double fraction = m_moveGroupPtr->computeCartesianPath(waypoints,
                                                           eefStep, 
                                                           jumpTreshold, 
                                                           trajectory);


    ROS_INFO ("Starting Cartesian path planning execution.");
    
    // Call planner, compute plan and visualize it
    moveit::planning_interface::MoveGroupInterface::Plan plannedPath; 

    bool tracIK = true; 
    // Remove first element if tracIK used for this 
    if (tracIK){
        //Nothing for now
    }

    plannedPath.trajectory_.joint_trajectory.header = trajectory.joint_trajectory.header; 
    plannedPath.trajectory_.joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names;
    plannedPath.trajectory_.multi_dof_joint_trajectory.header = trajectory.multi_dof_joint_trajectory.header;
    plannedPath.trajectory_.multi_dof_joint_trajectory.joint_names = trajectory.multi_dof_joint_trajectory.joint_names; 

    std::vector<int>::size_type size = trajectory.joint_trajectory.points.size(); 
    //ROS_INFO("Number of points for trajectory is: %i", size); 
    
    bool descartesPlanning = true; 
    // Remove first element (https://answers.ros.org/question/253004/moveit-problem-error-trajectory-message-contains-waypoints-that-are-not-strictly-increasing-in-time/)    
    if (descartesPlanning) {
        for (std::size_t i = 0; i < trajectory.joint_trajectory.points.size() ; ++i) {
            if ( i > 0 ) {
                plannedPath.trajectory_.joint_trajectory.points.push_back(trajectory.joint_trajectory.points[i]);   

            }
        } 
        for (std::size_t i = 0; i < trajectory.multi_dof_joint_trajectory.points.size() ; ++i) {   
            if ( i > 0 ) {
            plannedPath.trajectory_.multi_dof_joint_trajectory.points.push_back(trajectory.multi_dof_joint_trajectory.points[i]);            
            } 
        }    
    }else {
        plannedPath.trajectory_.joint_trajectory.points = trajectory.joint_trajectory.points; 
        plannedPath.trajectory_.multi_dof_joint_trajectory.points = trajectory.multi_dof_joint_trajectory.points; 
    }

    m_moveGroupPtr->execute(plannedPath); 

    sleep(15);

    return true; 

}

bool ControlArm::getIK(const std::size_t attempts, double timeout) {

    // Or use EndEffector link 
    Eigen::Affine3d currentPose_ = m_moveGroupPtr->getCurrentState()->getFrameTransform("lwa4p_link6");
    geometry_msgs::Pose currentROSPose_; 
    tf::poseEigenToMsg(currentPose_, currentROSPose_);

    bool found_ik = m_currentRobotStatePtr->setFromIK(m_jointModelGroupPtr, currentROSPose_);

    // publish current pose
    currentPosePublisher_.publish(currentROSPose_);

    bool debug = false; 
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

    ros::Rate r(25);

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

        if (!firstTrajectoryExecution_) {
            firstTrajectoryExecution_ =  executeDummyCartesianPath(); 
        }


        // Call get current end effector state to setup pointer of variable which is used in get Inverse Kinematics
        // getCurrentEndEffectorState("lwa4p_link6"); 

        // Call to get IK 
        std::size_t attempts = 10; 
        double timeout = 1; 
        bool successIK; 
        successIK = getIK(attempts, timeout); 

        //Eigen::MatrixXd m_; 
        //Eigen::Vector3d testVector(0.0, 0.0, 0.0);
        //m_ = getJacobian(testVector);
        //ROS_INFO_STREAM("Jacobian: \n" << m_); 
    
        r.sleep(); 
    }
}








