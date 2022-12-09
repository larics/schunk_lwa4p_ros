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
    nodeHandle_.param("init/sleep_time", sleepMs_, 10000000);
    nodeHandle_.param("start_christmas", startChristmas_, true);

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
    int currentJointCmdQueueSize;

    std::string disableCollisionServiceName;
    std::string addCollisionObjectServiceName;
    std::string startPositionControllersServiceName;
    std::string startJointTrajectoryControllerServiceName;
    std::string startJointGroupPositionControllerServiceName;
    std::string startJointGroupVelocityControllerServiceName;
    std::string sendArmToHomingPoseServiceName;
    std::string checkIKSolutionsServiceName;
    std::string executeCartesianPathServiceName;

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
    nodeHandle_.param("services/disable_collision_service", disableCollisionServiceName, std::string("tool/disable_collision"));
    nodeHandle_.param("services/add_collision_object", addCollisionObjectServiceName, std::string("scene/add_collisions"));
    nodeHandle_.param("services/start_position_controllers", startPositionControllersServiceName, std::string("controllers/start_position_controllers"));
    nodeHandle_.param("services/start_joint_trajectory_controller", startJointTrajectoryControllerServiceName, std::string("controllers/start_joint_trajectory_controller"));
    nodeHandle_.param("services/start_joint_group_position_controller", startJointGroupPositionControllerServiceName, std::string("controllers/start_joint_group_position_controller"));
    nodeHandle_.param("services/start_joint_group_velocity_controller", startJointGroupVelocityControllerServiceName, std::string("controllers/start_joint_group_velocity_controller"));
    nodeHandle_.param("services/send_arm_to_homing_pose", sendArmToHomingPoseServiceName, std::string("arm/send_arm_to_homing_pose"));
    nodeHandle_.param("services/check_ik_soutions", checkIKSolutionsServiceName, std::string("arm/check_ik_solutions"));
    nodeHandle_.param("services/execute_cartesian_path", executeCartesianPathServiceName, std::string("arm/execute_cartesian_path"));

    ROS_INFO("[ControlArm] Initializing subscribers/publishers..." );
    displayTrajectoryPublisher_ = nodeHandle_.advertise<moveit_msgs::DisplayTrajectory>(displayTrajectoryTopicName, displayTrajectoryQueueSize);
    currentPosePublisher_ = nodeHandle_.advertise<geometry_msgs::Pose>(currentPoseTopicName, currentPoseTopicQueueSize);
    cmdJoint1Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Float64>(std::string("lwa4p/joint_1_position_controller/command"), 1);
    cmdJoint2Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Float64>(std::string("lwa4p/joint_2_position_controller/command"), 1);
    cmdJoint3Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Float64>(std::string("lwa4p/joint_3_position_controller/command"), 1);
    cmdJoint4Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Float64>(std::string("lwa4p/joint_4_position_controller/command"), 1);
    cmdJoint5Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Float64>(std::string("lwa4p/joint_5_position_controller/command"), 1);
    cmdJoint6Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Float64>(std::string("lwa4p/joint_6_position_controller/command"), 1);
    cmdJointGroupPositionPublisher = nodeHandleWithoutNs_.advertise<std_msgs::Float64MultiArray>(std::string("lwa4p/joint_group_position_controller/command"), 1);
    cmdJointGroupVelocityPublisher = nodeHandleWithoutNs_.advertise<std_msgs::Float64MultiArray>(std::string("lwa4p/joint_group_velocity_controller/command"), 1);
    powerline0PosePublisher = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/powerline0_pose",1);
    powerline1PosePublisher = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/powerline1_pose",1);

    armCmdPoseSubscriber_ = nodeHandle_.subscribe<geometry_msgs::Pose>(cmdPoseTopicName, cmdPoseTopicQueueSize, &ControlArm::cmdPoseCallback, this);
    armCmdToolOrientationSubscriber_ = nodeHandle_.subscribe<geometry_msgs::Point>(cmdToolOrientationTopicName, cmdToolOrientationTopicQueueSize, &ControlArm::cmdToolOrientationCallback, this); 
    armCmdDeltaPoseSubscriber_ = nodeHandle_.subscribe<geometry_msgs::Pose>(cmdDeltaPoseTopicName, cmdDeltaPoseTopicQueueSize, &ControlArm::cmdDeltaPoseCallback, this); 
    ROS_INFO("[ControlArm] Initialized subscribers/publishers.");

    // Initialize Services
    ROS_INFO("[ControlArm] Initializing services...");
    disableCollisionService_ = nodeHandle_.advertiseService(disableCollisionServiceName, &ControlArm::disableCollisionServiceCallback, this);
    addCollisionObjectService_ = nodeHandle_.advertiseService(addCollisionObjectServiceName, &ControlArm::addCollisionObjectServiceCallback, this);
    startPositionControllersService_ = nodeHandle_.advertiseService(startPositionControllersServiceName, &ControlArm::startPositionControllers, this);
    startJointTrajectoryControllerService_ = nodeHandle_.advertiseService(startJointTrajectoryControllerServiceName, &ControlArm::startJointTrajectoryController, this);
    startJointGroupPositionControllerService_ = nodeHandle_.advertiseService(startJointGroupPositionControllerServiceName, &ControlArm::startJointGroupPositionController, this);
    startJointGroupVelocityControllerService_ = nodeHandle_.advertiseService(startJointGroupVelocityControllerServiceName, &ControlArm::startJointGroupVelocityController, this);
    sendArmToHomingPoseService_ = nodeHandle_.advertiseService(sendArmToHomingPoseServiceName, &ControlArm::sendArmToHomingPose, this);
    checkIKSolutionsService_ = nodeHandle_.advertiseService(checkIKSolutionsServiceName, &ControlArm::checkIKSolutionsServiceCallback, this);
    executeCartesianPathService_ = nodeHandle_.advertiseService(executeCartesianPathServiceName, &ControlArm::executeCartesianServiceCallback, this);
    ROS_INFO("[ControlArm] Initialized services.");

    // Initialize Clients for other services
    ROS_INFO("[ControlArm] Initializing service clients...");
    applyPlanningSceneServiceClient_ = nodeHandleWithoutNs_.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
    applyPlanningSceneServiceClient_.waitForExistence();
    switchControllerServiceClient_ = nodeHandleWithoutNs_.serviceClient<controller_manager_msgs::SwitchController>("lwa4p/controller_manager/switch_controller");
    switchControllerServiceClient_.waitForExistence();
    listControllersServiceClient_ = nodeHandleWithoutNs_.serviceClient<controller_manager_msgs::ListControllers>("lwa4p/controller_manager/list_controllers");
    listControllersServiceClient_.waitForExistence();
    switchToPositionControllerServiceClient_= nodeHandle_.serviceClient<std_srvs::Trigger>(startPositionControllersServiceName);
    switchToTrajectoryControllerServiceClient_ = nodeHandle_.serviceClient<std_srvs::Trigger>(startJointTrajectoryControllerServiceName);
    getIKSolutionsServiceClient_ = nodeHandle_.serviceClient<moveit_msgs::GetPositionIKRequest>(checkIKSolutionsServiceName);
    getIKSolutionsServiceClient_.waitForExistence();

    gripperGraspServiceClient_ = nodeHandleWithoutNs_.serviceClient<wsg_50_common::Move>("/wsg_50_driver/grasp");
    gripperMoveServiceClient_ = nodeHandleWithoutNs_.serviceClient<wsg_50_common::Move>("/wsg_50_driver/move");
    gripperSetForceServiceClient_ = nodeHandleWithoutNs_.serviceClient<wsg_50_common::Conf>("/wsg_50_driver/set_force");
    gripperReleaseServiceClient_ = nodeHandleWithoutNs_.serviceClient<wsg_50_common::Move>("/wsg_50_driver/release");



    if (startChristmas_) {

        homingService_ = nodeHandleWithoutNs_.advertiseService("/schunk_go_home",
                                                               &ControlArm::schunkHomingServiceCallback, this);

        getDrinkService_ = nodeHandleWithoutNs_.advertiseService("/schunk_get_drink", &ControlArm::schunkGetDrinkServiceCallback, this);
        leaveDrinkService_ = nodeHandleWithoutNs_.advertiseService("/schunk_serve_drink", &ControlArm::schunkLeaveDrinkServiceCallback, this);

        schunkGetDrinkDonePublisher_ = nodeHandleWithoutNs_.advertise<std_msgs::Bool>("/schunk_get_drink_done", 1);
        schunkServeDrinkDonePublisher_ = nodeHandleWithoutNs_.advertise<std_msgs::Bool>("/schunk_serve_drink_done", 1);

        // Christmas publishers
        schunkOrderSubscriber = nodeHandleWithoutNs_.subscribe<std_msgs::Bool>("/schunk_order_reciv", 1,
                                                                               &ControlArm::orderCallback, this);
        schunkOrderPublisher = nodeHandleWithoutNs_.advertise<std_msgs::Bool>("/schunk_order_done", 1);

    }

    //addCollisionObjectServiceClient_ = nodeHandle_.serviceClient<std_srvs::Trigger>("scene/add_collisions");
    ROS_INFO("[ControlArm] Initialized service clients. ");



}


bool ControlArm::schunkGetDrinkServiceCallback(christmas_fair_common::StartTrajectorySrvRequest &req, christmas_fair_common::StartTrajectorySrvResponse &res){

    m_getDrink = true;
    return true;

}

bool ControlArm::schunkLeaveDrinkServiceCallback(christmas_fair_common::StartTrajectorySrvRequest &req, christmas_fair_common::StartTrajectorySrvResponse &res){

    m_leaveDrink = true;
    return true;

}



bool ControlArm::setMoveGroup() {

    ROS_INFO("[ControlArm] Setting move group." );

    // MoveIt move group
    static const std::string groupName = "arm"; 
    m_moveGroupPtr = new moveit::planning_interface::MoveGroupInterface(groupName); 

    // Allow replanning 
    m_moveGroupPtr->allowReplanning(true);

    // Set end effector frame
    m_moveGroupPtr->setEndEffectorLink("wsg50_center");

    int planningTime = 10;
    m_moveGroupPtr->setPlanningTime(planningTime);
    m_moveGroupPtr->setNumPlanningAttempts(10);



    // Get current robot arm state
    getCurrentArmState(); 

    return true;

}

bool ControlArm::setPlanningScene() {

    ROS_INFO("[ControlArm] Setting planning scene.");

    // MoveIt planning scene setup as seen (http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/planning_scene/planning_scene_tutorial.html)
    robot_model_loader::RobotModelLoader m_robotLoader("robot_description");
    const moveit::core::RobotModelPtr& kinematic_model = m_robotLoader.getModel();
    m_planningScenePtr = new planning_scene::PlanningScene(kinematic_model);

    ROS_INFO("[ControlArm] Model frame: %s", kinematic_model->getModelFrame().c_str());
    return true; 
}

// This is wrong, we should pass cmdPose as argument into function and then set it if we plan to use setters
bool ControlArm::setCmdPose() {

    if (moveGroupInitialized_) {

        m_moveGroupPtr->setPoseTarget(m_cmdPose);
        return true;
    }
    return false;

}

bool ControlArm::setCmdJoint() {

    ROS_INFO_STREAM("m_cmdJoint is: " << m_cmdJoint);
    if (moveGroupInitialized_){
        m_moveGroupPtr->setJointValueTarget(m_cmdJoint.position);
        return true;
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

    float q_x = msg->orientation.x;
    float q_y = msg->orientation.y;
    float q_z = msg->orientation.z;
    float q_w = msg->orientation.w;

    float norm = std::sqrt(std::pow(q_x, 2) + std::pow(q_y, 2) + std::pow(q_z, 2) + std::pow(q_w, 2));

    float q_x_normalized = q_x / norm;
    float q_y_normalized = q_y / norm;
    float q_z_normalized = q_z / norm;
    float q_w_normalized = q_w / norm;

    m_cmdPose.orientation.x = q_x_normalized;
    m_cmdPose.orientation.y = q_y_normalized;
    m_cmdPose.orientation.z = q_z_normalized;
    m_cmdPose.orientation.w = q_w_normalized;

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

    m_moveGroupPtr->setMaxVelocityScalingFactor(0.1);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(0.1);

    getCurrentArmState();

    // Update current joint values --> This should solve problem of deviates from current state
    m_moveGroupPtr->setStartStateToCurrentState();

    // Call inverse kinematics when pose is commanded
    bool found_ik = getIK(10, 1);

    //m_currentRobotStatePtr->getJointPositions(plannedPath.start_state_.joint_state);

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

bool ControlArm::sendToCmdJoint(){
    setCmdJoint();

    moveit::planning_interface::MoveGroupInterface::Plan plannedPath;

    // TODO: Add velocity scaling factor as params
    m_moveGroupPtr->setMaxVelocityScalingFactor(0.1);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(0.1);

    // Set start state
    getCurrentArmState();

    // Update current joint values --> This should solve problem of deviates from current state
    m_moveGroupPtr->setStartStateToCurrentState();

    //m_currentRobotStatePtr->getJointPositions(plannedPath.start_state_.joint_state);

    // plan Path
    bool success = (m_moveGroupPtr->plan(plannedPath) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    m_moveGroupPtr->execute(plannedPath);


    ROS_INFO("[ControlArm] Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");


    return success;
}

bool ControlArm::planToCmdPose(){
    setCmdPose();

    // Call planner, compute plan and visualize it
    moveit::planning_interface::MoveGroupInterface::Plan plannedPath;

    m_moveGroupPtr->setMaxVelocityScalingFactor(0.1);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(0.1);

    //TODO: Set start state :) as it could be correctly

    getCurrentArmState();

    // Update current joint values --> This should solve problem of deviates from current state
    m_moveGroupPtr->setStartStateToCurrentState();

    //m_currentRobotStatePtr->getJointPositions(plannedPath.start_state_.joint_state);

    // plan Path
    bool success = (m_moveGroupPtr->plan(plannedPath) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    return success;
}

bool ControlArm::executeMovement(){

    if (blockingMovement){
        m_moveGroupPtr->move();
    }else {
        m_moveGroupPtr->asyncMove();
    }

}

void ControlArm::sendToCmdPoses(std::vector<geometry_msgs::Pose> poses)
{

    getJointPositions(m_jointModelGroupPtr->getVariableNames(), m_jointPositions_);

    ROS_INFO_STREAM("Entered sendToCmdPoses!");
    ROS_INFO_STREAM("Poses size: " << poses.size());

    for (int i = 0; i < poses.size(); ++i)
    {

        ROS_INFO_STREAM("[ControlArmNode] Visiting pose " << i);
        m_cmdPose.position = poses.at(i).position;
        m_cmdPose.orientation = poses.at(i).orientation;
        sendToCmdPose();
        //if (m_moveGroupPtr->state)
    }



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

    return true;

}

void ControlArm::addCollisionObject(moveit_msgs::PlanningScene& planningScene){

    ROS_INFO("Adding collision object...");

    std::vector<moveit_msgs::CollisionObject> collisionObjects;
    moveit_msgs::CollisionObject collisionObject1; moveit_msgs::CollisionObject collisionObject2;

    collisionObject1.header.frame_id = m_moveGroupPtr->getPlanningFrame();
    collisionObject2.header.frame_id = m_moveGroupPtr->getPlanningFrame();

    shape_msgs::SolidPrimitive primitive;

    collisionObject2.id = "wall";
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 3.0;
    primitive.dimensions[2] = 2.0;

    geometry_msgs::Pose wall_pose;
    wall_pose.orientation.w = 0.707;
    wall_pose.orientation.z = 0.707;
    wall_pose.position.x = -0.70;
    wall_pose.position.y = -0.70;
    wall_pose.position.z = 1.0;

    collisionObject2.primitives.push_back(primitive);
    collisionObject2.primitive_poses.push_back(wall_pose);
    collisionObject2.operation = collisionObject2.ADD;

    collisionObjects.push_back(collisionObject2);

    if(startChristmas_){
        moveit_msgs::CollisionObject collisionObject3;
        collisionObject3.header.frame_id = m_moveGroupPtr->getPlanningFrame();

        shape_msgs::SolidPrimitive primitive1;
        primitive1.type = primitive.BOX;
        primitive1.dimensions.resize(3);
        primitive1.dimensions[0] = 0.38;
        primitive1.dimensions[1] = 0.38;
        primitive1.dimensions[2] = 0.45;

        geometry_msgs::Pose object_pose;
        object_pose.orientation.w = 1.0;
        object_pose.position.x = 0.0;
        object_pose.position.y = -0.6;
        object_pose.position.z = 0.225;
        collisionObject3.primitives.push_back(primitive1);
        collisionObject3.primitive_poses.push_back(object_pose);
        collisionObject3.operation = collisionObject3.ADD;

        collisionObjects.push_back(collisionObject3);



    }

    for(std::size_t i = 0; i < collisionObjects.size(); ++i){
        planningScene.world.collision_objects.push_back(collisionObjects.at(i));
    };


    ROS_INFO("Added collisions");



}

bool ControlArm::checkIKSolutionsServiceCallback(moveit_msgs::GetPositionIKRequest &req, moveit_msgs::GetPositionIKResponse &res){


    //moveit_msgs::GetPositionIKRequest ik_req; moveit_msgs::GetPositionIKResponse ik_res;
    //sensor_msgs::JointState current_joint_state;

    //std::vector<std::string> joint_names;
    //std::vector<double> joint_values;

    //joint_names = m_moveGroupPtr->getJointNames();
    //joint_values = m_moveGroupPtr->getCurrentJointValues();

    //ROS_INFO_STREAM("Joint values: " << joint_values);
    //ROS_INFO_STREAM("Joint names: " << joint_names);

    //current_joint_state.name = joint_names;
    //current_joint_state.position = joint_values;

    //ik_req.ik_request.group_name = m_moveGroupPtr->getName();
    //ik_req.ik_request.robot_state.joint_state = current_joint_state;
    //ik_req.ik_request.ik_link_name = m_moveGroupPtr->getEndEffectorLink();
    //ik_req.ik_request.avoid_collisions = true;

    geometry_msgs::PoseStamped ik_pose;
    ik_pose = req.ik_request.pose_stamped;
    req.ik_request.pose_stamped = ik_pose;

    ROS_INFO_STREAM("Calling IK check service: " << req);

    geometry_msgs::Pose pose;
    pose = req.ik_request.pose_stamped.pose;

    getCurrentArmState();

    bool found_ik = m_currentRobotStatePtr->setFromIK(m_jointModelGroupPtr, pose,0.1);
    ROS_INFO_STREAM("Found IK solution for wanted pose: " << found_ik);
    std::vector<double> joint_values;
    if (found_ik){
        m_currentRobotStatePtr->copyJointGroupPositions(m_jointModelGroupPtr, joint_values);
    }
    if (found_ik){
        for(std::size_t i = 0; i < joint_values.size(); ++i)
        {
            ROS_INFO("Joint %i: %f", i, joint_values[i]);
        }
    }

    return found_ik;
}

bool ControlArm::disableCollisionServiceCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res){
    // TODO: Move this to specific script because it's related to magnetic localization

    if (planningSceneInitialized_){
        collision_detection::AllowedCollisionMatrix acm = m_planningScenePtr->getAllowedCollisionMatrix();

        // Before setting collisions
        acm.setEntry("powerline_cable1", "separator_right_head", true);
        acm.setEntry("powerline_cable1", "separator_left_head", true);
        acm.setEntry("powerline_cable1", "separator_main", true);
        acm.setEntry("powerline_cable2", "separator_right_head", true);
        acm.setEntry("powerline_cable2", "separator_left_head", true);
        acm.setEntry("powerline_cable2", "separator_main", true);
        acm.setEntry("powerline_cable2", "separator_base", true);

        moveit_msgs::PlanningScene planningScene;
        m_planningScenePtr->getPlanningSceneMsg(planningScene);
        // Create new collision matrix
        acm.getMessage(planningScene.allowed_collision_matrix);
        planningScene.is_diff = true;
        //m_planningScenePtr->setPlanningSceneMsg(planningScene); --> Setting it over mPlanningScenePtr;

        moveit_msgs::ApplyPlanningScene srv;
        srv.request.scene = planningScene;
        applyPlanningSceneServiceClient_.call(srv);

        bool debugOut = false;
        if(debugOut){
            acm.print(std::cout);
            ROS_INFO("[ControlArm] Disabled collisions: %d", (bool) srv.response.success);
            collision_detection::AllowedCollisionMatrix acm_after = m_planningScenePtr->getAllowedCollisionMatrix();
            acm_after.print(std::cout);
        }
        return true;
    }
    else{
        return false;
    }

}

void ControlArm::getRunningControllers(std::vector<std::string> &runningControllerNames){
    ROS_INFO("[ControlArm] Listing controllers: ");
    controller_manager_msgs::ListControllersRequest listReq; controller_manager_msgs::ListControllersResponse listRes;
    listControllersServiceClient_.call(listReq, listRes);
    //ROS_INFO_STREAM("[ControlArm] Controllers: " << listRes);

    for(std::size_t i = 0; i < listRes.controller.size(); ++i){
        if (listRes.controller[i].state == "running" ){
            // Additional constraints for controllers that must be active all of the time
            if(listRes.controller[i].name != "joint_state_controller" && listRes.controller[i].name != "distancer_right_position_controller" && listRes.controller[i].name != "distancer_left_position_controller")
            {
                runningControllerNames.push_back(listRes.controller[i].name);

                ROS_INFO_STREAM("[ControlArm] Stopping controller: " << listRes.controller[i].name);
            }
        }
    }

    //ROS_INFO_STREAM("[ControlArm] Running controllers are: " << runningControllerNames);

}

bool ControlArm::startJointTrajectoryController(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

    std::vector<std::string> runningControllers; getRunningControllers(runningControllers);

    ROS_INFO("[ControlArm] Starting JointTrajectoryController...");
    // Stop running controllers
    controller_manager_msgs::SwitchControllerRequest switchControllerRequest;
    controller_manager_msgs::SwitchControllerResponse switchControllerResponse;
    for(std::size_t i = 0; i < runningControllers.size(); ++i){
        switchControllerRequest.stop_controllers.push_back(runningControllers[i]);
    }
    switchControllerRequest.start_controllers.push_back(std::string("arm_controller"));
    switchControllerRequest.start_asap = true;
    //ontroller Manager: To switch controllers you need to specify a strictness level of
    // controller_manager_msgs::SwitchController::STRICT (2) or ::BEST_EFFORT (1). Defaulting to ::BEST_EFFORT.
    switchControllerRequest.strictness = 2;
    switchControllerRequest.timeout = 10;

    switchControllerServiceClient_.call(switchControllerRequest, switchControllerResponse);

    return switchControllerResponse.ok;

}

bool ControlArm::startJointGroupPositionController(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

    std::vector<std::string> runningControllers; getRunningControllers(runningControllers);

    ROS_INFO("[ControlArm] Starting JointGroupPositionController...");
    controller_manager_msgs::SwitchControllerRequest switchControllerRequest;
    controller_manager_msgs::SwitchControllerResponse  switchControllerResponse;
    // Stop running controllers
    for(std::size_t i = 0; i < runningControllers.size(); ++i){
        switchControllerRequest.stop_controllers.push_back(runningControllers[i]);
    }
    switchControllerRequest.start_controllers.push_back(std::string("joint_group_position_controller"));
    switchControllerRequest.start_asap = true;
    switchControllerRequest.strictness = 2;
    switchControllerRequest.timeout = 10;

    switchControllerServiceClient_.call(switchControllerRequest, switchControllerResponse);
    ros::Duration(0.5).sleep();
    // TODO: Add enabling stuff for different controller type
    ROS_INFO("Sending all joints to zero");

    //activateJoints();
    sendZeros("group"); // Enables sending of commands to jointGroupController
    ros::Duration(0.5).sleep();
    //TODO: Add method for sending current joint states


    return switchControllerResponse.ok;
}

bool ControlArm::startJointGroupVelocityController(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

    std::vector<std::string> runningControllers; getRunningControllers(runningControllers);

    ROS_INFO("[ControlArm] Start JointGroupVelocityController...");
    controller_manager_msgs::SwitchControllerRequest switchControllerRequest;
    controller_manager_msgs::SwitchControllerResponse switchControllerResponse;
    // Stop running controllers
    for(std::size_t i = 0; i < runningControllers.size(); ++i){
        switchControllerRequest.stop_controllers.push_back(runningControllers[i]);
    }
    switchControllerRequest.start_controllers.push_back(std::string("joint_group_velocity_controller"));
    switchControllerRequest.start_asap = true;
    switchControllerRequest.strictness = 2;
    switchControllerRequest.timeout = 10;

    switchControllerServiceClient_.call(switchControllerRequest, switchControllerResponse);

    ROS_INFO("Switched to velocity controller");

    return switchControllerResponse.ok;



}

bool ControlArm::startPositionControllers(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

    std::vector<std::string> runningControllers; getRunningControllers(runningControllers);

    ROS_INFO("[ControlArm] Starting JointPosition controllers...");
    controller_manager_msgs::SwitchControllerRequest switchControllerRequest;
    controller_manager_msgs::SwitchControllerResponse switchControllerResponse;
    // Stop running controllers
    for(std::size_t i = 0; i < runningControllers.size(); ++i){
        switchControllerRequest.stop_controllers.push_back(runningControllers[i]);
    }
    switchControllerRequest.start_controllers.push_back(std::string("joint_1_position_controller"));
    switchControllerRequest.start_controllers.push_back(std::string("joint_2_position_controller"));
    switchControllerRequest.start_controllers.push_back(std::string("joint_3_position_controller"));
    switchControllerRequest.start_controllers.push_back(std::string("joint_4_position_controller"));
    switchControllerRequest.start_controllers.push_back(std::string("joint_5_position_controller"));
    switchControllerRequest.start_controllers.push_back(std::string("joint_6_position_controller"));
    switchControllerRequest.start_asap = true;
    // Controller Manager: To switch controllers you need to specify a strictness level of
    // controller_manager_msgs::SwitchController::STRICT (2) or ::BEST_EFFORT (1). Defaulting to ::BEST_EFFORT.
    switchControllerRequest.strictness = 2;
    switchControllerRequest.timeout = 10;

    switchControllerServiceClient_.call(switchControllerRequest, switchControllerResponse);
    ros::Duration(0.1).sleep();

    //sendZeros("position");

    return switchControllerResponse.ok;

}

bool ControlArm::addCollisionObjectServiceCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res){

    ROS_INFO("Entered collision object");
    // Initialize planning scene
    moveit_msgs::PlanningScene planningScene;
    m_planningScenePtr->getPlanningSceneMsg(planningScene);
    ROS_INFO("Got planning scene.");
    addCollisionObject(planningScene);

    //
    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planningScene;
    applyPlanningSceneServiceClient_.call(srv);

    return true; 

    // How to add this to planning scene moveit?
    // http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/planning_scene_ros_api/planning_scene_ros_api_tutorial.html
}

bool ControlArm::sendArmToHomingPose(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res){

    ROS_INFO("[ControlArm] Sending arm to home position.");

    // HARDCODED POSE SUITABLE FOR CURRENT SCHUNK ARM CONFIGURATION
    geometry_msgs::Pose homingPose;
    homingPose.position.x = 0.1;
    homingPose.position.y = 0.1;
    homingPose.position.z = 1.05;
    homingPose.orientation.x = 0;
    homingPose.orientation.y = 0;
    homingPose.orientation.z = 0;
    homingPose.orientation.w = 1;

    m_cmdPose = homingPose;
    sendToCmdPose();

    // TODO: Add sending to 0 of each joint
    // Switch to position controller and send commands for each joint to reach 0 configuration of arm
    ros::Duration(15).sleep();

    // Call Service to enable position control for each joint
    std_srvs::Trigger srv;
    switchToPositionControllerServiceClient_.call(srv);

    ros::Duration(1).sleep();

    std::vector<std::string> jointNames = m_jointModelGroupPtr->getVariableNames();
    std::vector<double> currentJointPositions_;

    // Get current joint positions ( send every joint to 0)
    getJointPositions(jointNames, currentJointPositions_);
    // Send zeros to enable joints
    for (std::size_t i = jointNames.size() - 1 ; i + 1 >0 ; --i){

        std_msgs::Float64 jointCmd_;
        std_msgs::Float64 jointCmd1_; jointCmd1_.data = 0;
        jointCmd_.data = - currentJointPositions_[i];
        //jointCmd_.data = 0;
        // Doesn't work as it should, something weird is happening,
        // Either I get an emergency error
        // Either it doesn't rotate
        // Sometimes it rotates
        // Wierd behaviour
        ROS_INFO("[ControlArm] Sending joint %s to  - %f", jointNames[i].c_str(), currentJointPositions_[i]);


        if (i == 5){
            for (int k=0; k<5; k++)
            {
                cmdJoint6Publisher.publish(jointCmd_);
                ros::Duration(0.02).sleep();
            }
        }
        if (i == 4){
            for (int k=0; k<5; k++)
            {
                cmdJoint5Publisher.publish(jointCmd_);
                ros::Duration(0.02).sleep();

            }
        }
        if (i == 3){

            for (int k=0; k<5; k++)
            {
                cmdJoint4Publisher.publish(jointCmd_);
                ros::Duration(0.02).sleep();

            }

        }
        if (i == 2){

            for (int k=0; k<5; k++)
            {
                cmdJoint3Publisher.publish(jointCmd_);
                ros::Duration(0.02).sleep();
            }

        }
        if (i == 1){
            for (int k=0; k<5; k++)
            {
                cmdJoint2Publisher.publish(jointCmd_);
                ros::Duration(0.02).sleep();
            }

        }
        if (i == 0){
            for (int k=0; k<5; k++)
            {
                cmdJoint1Publisher.publish(jointCmd1_);
                ros::Duration(0.02).sleep();
            }
        }

        float sleep_t = abs ( currentJointPositions_[i] / 0.1 ) ; // MAX_SPEED RAD/S
        ros::Duration(sleep_t).sleep();
    }


    return true;

}

bool ControlArm::sendZeros(std::string ControllerType){
    // NOTE: This method is used to activate joint control for each arm joint.
    // Joints do not move before recieving 0 as command, after recieving 0, you can send
    // anything and it should be fine --> Check schunk documentation for this

    if (ControllerType == "position"){

        std_msgs::Float64 msg;
        msg.data = 0;

        //cmdJoint1Publisher.publish(msg);
        cmdJoint2Publisher.publish(msg);
        cmdJoint3Publisher.publish(msg);
        cmdJoint4Publisher.publish(msg);
        cmdJoint5Publisher.publish(msg);
        cmdJoint6Publisher.publish(msg);

    }
    if (ControllerType == "group"){
        std_msgs::Float64MultiArray msg;

        std::vector<double> activationPositions = {m_jointPositions_[0],
                                             m_jointPositions_[1],
                                             m_jointPositions_[2],
                                             m_jointPositions_[3],
                                             m_jointPositions_[4],
                                             0};

        msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        msg.layout.dim[0].size = activationPositions.size();
        msg.layout.dim[0].stride = 1;
        msg.layout.dim[0].label = "i";

        msg.data.clear();
        msg.data.insert(msg.data.end(), activationPositions.begin(), activationPositions.end());

        cmdJointGroupPositionPublisher.publish(msg);

    }

    return true;
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

void ControlArm::getJointPositions(const std::vector<std::string>& jointNames, std::vector<double> &jointGroupPositions) {

    m_currentRobotStatePtr->copyJointGroupPositions(m_jointModelGroupPtr, jointGroupPositions);

    bool debug = false; 
    if (debug){
        for (std::size_t i = 0; i < jointNames.size(); ++i)
        {
            ROS_INFO("Joint %s: %f", jointNames[i].c_str(), jointGroupPositions[i]);
        }

    }
  
}

bool ControlArm::executeCartesianServiceCallback(schunk_lwa4p_control::CartesianPath::Request &req, schunk_lwa4p_control::CartesianPath::Response &res)
{

    ROS_INFO_STREAM("Request is: " << req);

    //for(int i=0; i<req.waypoints.size())

    m_moveGroupPtr->setMaxVelocityScalingFactor(0.2);
    double eefStep = 0.01; double jumpThreshold = 0.01;
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = m_moveGroupPtr->computeCartesianPath(req.waypoints,
                                                           eefStep,
                                                           jumpThreshold,
                                                           trajectory);

    //stuff for cartesian path planning
    //if (cartesian){
    //    if(first){
    //        points.push_back(currentPose);
    //       geometry_msgs::Pose mid_pose;
    //      int num_points = 10;
    //     for (int i=0; i<num_points; ++i){
    /*        geometry_msgs::Pose tmp_pose;
            tmp_pose.position = currentPose.position; mid_pose.orientation = currentPose.orientation;
            tmp_pose.position.z = currentPose.position.z + i * (target_pose.pose.position.z - currentPose.position.z)/num_points;
            points.push_back(tmp_pose);
        }
        points.push_back(target_pose.pose);
        first = false;
        schunk_lwa4p_control::CartesianPath cp;
        cp.request.waypoints = points;
        executeCartesianPathClient_.call(cp);
    }
    //ROS_INFO_STREAM("Cartesian path execution:" << cp.response.succedded);
}*/




    ROS_INFO_NAMED("tutorial", "Visualizing plan  (Cartesian path) (%.2f%% acheived)", fraction * 100.0);


    //m_moveGroupPtr->execute(plannedPath);

    return true;



}

bool ControlArm::executeDummyCartesianPath(){
    geometry_msgs::Pose startPose = m_moveGroupPtr->getCurrentPose().pose; 
    
    std::vector<geometry_msgs::Pose> waypoints;
    startPose.position.z += 0.35;
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
    // TODO --> use end effector link -> wsg50_center

    // TODO: Get current joint values
    // ROS_INFO_STREAM(m_moveGroupPtr->getCurrentJointValues());

    // http://docs.ros.org/en/jade/api/joint_limits_interface/html/c++/index.html --> joint limits

    // https://www.ri.cmu.edu/pub_files/pub4/stilman_michael_2006_4/stilman_michael_2006_4.pdf

    Eigen::Affine3d currentPose_ = m_moveGroupPtr->getCurrentState()->getFrameTransform("wsg50_center");
    geometry_msgs::Pose currentROSPose_; 
    tf::poseEigenToMsg(currentPose_, currentROSPose_);


    bool debug = false;
    if (debug == true){
        ROS_INFO_STREAM("Current joint values target: " << m_moveGroupPtr->getJointValueTarget());
        ROS_INFO_STREAM("Current pose target is: " << m_moveGroupPtr->getPoseTarget());
    }

    // https://github.com/ros-planning/moveit/issues/2491
    bool found_ik = m_currentRobotStatePtr->setFromIK(m_jointModelGroupPtr, currentROSPose_);

    if (debug == true){
        ROS_INFO("Found IK solution!");
        ROS_INFO_STREAM("FOUND IK: " << found_ik);
    }

    // publish current pose
    currentPosePublisher_.publish(currentROSPose_);




    
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


float ControlArm::round(float var){

    float value = (int)(var * 1000 + .5);
    return (float) value/1000;

}

bool ControlArm::schunkHomingServiceCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res){
    m_homingSchunk = true;

    res.success = true;

    return true;
}

void ControlArm::orderCallback(const std_msgs::Bool::ConstPtr &msg) {

    // Recieved schunk order
    orderReciv = msg->data;
}

bool ControlArm::graspCup(int desiredForceN, int desiredWidthMM, int desiredSpeedMM)
{
    wsg_50_common::Conf setForceSrv;
    wsg_50_common::Move graspSrv;

    setForceSrv.request.val = desiredForceN;

    gripperSetForceServiceClient_.call(setForceSrv.request,
                                       setForceSrv.response);

    ROS_INFO_STREAM("Grasping cup! ");

    graspSrv.request.width = desiredWidthMM;
    graspSrv.request.speed = desiredSpeedMM;
    gripperGraspServiceClient_.call(graspSrv.request,
                                    graspSrv.response);

    return graspSrv.response.error;
}

bool ControlArm::releaseCup(int releaseWidthMM)
{
    wsg_50_common::Move graspSrv;

    graspSrv.request.width = releaseWidthMM;
    gripperGraspServiceClient_.call(graspSrv.request,
                                    graspSrv.response);

    return graspSrv.response.error;

}

void ControlArm::run() {

    ros::Rate r(25);
    bool executed = false;

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
        getJointPositions(m_jointModelGroupPtr->getVariableNames(), m_jointPositions_);

        if (!firstTrajectoryExecution_) {
            firstTrajectoryExecution_ =  executeDummyCartesianPath(); 
        }


        // Call get current end effector state to setup pointer of variable which is used in get Inverse Kinematics
        // getCurrentEndEffectorState("lwa4p_link6"); 

        // Call to get IK 
        std::size_t attempts = 10; 
        double timeout = 2;
        bool successIK;

        successIK = getIK(attempts, timeout);

        bool grasped = 0;

        if (startChristmas_){

            wsg_50_common::Move moveSrv;

            // WORKS OK!
            // TODO: Fix service type (enable integer passing to choose pick place)
            // TODO: Check pick place for gripper

            m_moveGroupPtr->setMaxVelocityScalingFactor(0.5);

            moveSrv.request.width = 110;
            moveSrv.request.speed = 20;
            gripperMoveServiceClient_.call(moveSrv.request,
                                           moveSrv.response);
            if (m_first){
                ROS_INFO_STREAM("Prepare for CUP picking up");
                // Prepare for CUP picking up
                m_cmdPose.position.x = -0.21999351791850036;
                m_cmdPose.position.y =  0.07994923839009373;
                m_cmdPose.position.z = 0.4000228469249295;
                m_cmdPose.orientation.x = -0.48228078379161843;
                m_cmdPose.orientation.y = 0.2814110443636556;
                m_cmdPose.orientation.z = 0.6925185927769711;
                m_cmdPose.orientation.w = 0.45676150051608233;
                sendToCmdPose();

                m_first = false;

            }


            if (m_getDrink)
            {

                ROS_INFO_STREAM("Picking up CUP!");
                m_cmdPose.position.x =  -0.2796451906994132;
                m_cmdPose.position.y = 0.16182028293086587;
                m_cmdPose.position.z = 0.42983691307518224;
                m_cmdPose.orientation.x = -0.4827742754366359;
                m_cmdPose.orientation.y = 0.2810980331084555;
                m_cmdPose.orientation.z = 0.6924056981982725;
                m_cmdPose.orientation.w = 0.4566040339963634;
                sendToCmdPose();

                m_moveGroupPtr->setMaxVelocityScalingFactor(0.1);
                ROS_INFO_STREAM("Grasping CUP!");
                grasped = graspCup(1, 70, 10);

                m_cmdPose.position.x = -0.26857053829029204;
                m_cmdPose.position.y = 0.12750812186720534;
                m_cmdPose.position.z = 0.5178119215703323;
                sendToCmdPose();

                ROS_INFO_STREAM("Moving CUP for tanking");

                m_cmdPose.position.x =  -0.38903250589363286;
                m_cmdPose.position.y = -0.02198872538898188;
                m_cmdPose.position.z =  0.5278316529083782;
                m_cmdPose.orientation.x = 0.700305030404181;
                m_cmdPose.orientation.y = -0.003283744542555414;
                m_cmdPose.orientation.z = -0.7137208368642306;
                m_cmdPose.orientation.w = 0.012831540757123163;
                sendToCmdPose();

                ROS_INFO_STREAM("Drink getting poured in CUP!");

                m_cmdPose.position.x = -0.46327444415345753;
                m_cmdPose.position.y = -0.03183773996215472;
                m_cmdPose.position.z = 0.5295631418123471;
                sendToCmdPose();

                m_getDrink = false;
                std_msgs::Bool successMsg;
                successMsg.data = true;

                schunkGetDrinkDonePublisher_.publish(successMsg);


            }

            if (m_leaveDrink) {


                m_cmdPose.position.x =  -0.38903250589363286;
                m_cmdPose.position.y = -0.02198872538898188;
                m_cmdPose.position.z =  0.5278316529083782;
                sendToCmdPose();

                m_cmdPose.position.x =  -0.29151825795922875;
                m_cmdPose.position.y = -0.2126715159860169;
                m_cmdPose.position.z = 0.48817147602771543;
                m_cmdPose.orientation.x = -0.6423008269885361;
                m_cmdPose.orientation.y = -0.2388613807318374;
                m_cmdPose.orientation.z = 0.6812359300828926;
                m_cmdPose.orientation.w = -0.25751212788686095;
                sendToCmdPose();

                m_cmdPose.position.x = -0.11005869639614922;
                m_cmdPose.position.y = -0.4210141033383795;
                m_cmdPose.position.z = 0.5600720571198052;
                m_cmdPose.orientation.x = 0.472640346696874;
                m_cmdPose.orientation.y = 0.47220499819144507;
                m_cmdPose.orientation.z = -0.5228792082985984;
                m_cmdPose.orientation.w = 0.5293683744674486;
                sendToCmdPose();

                m_cmdPose.position.z = 0.5200720571198052;
                sendToCmdPose();

                ROS_INFO_STREAM("Releasing DRINK!");
                // wait for sipping to finish
                bool released = releaseCup(110);
                ros::Duration(1.0).sleep();

                m_cmdPose.position.z = 0.57;
                sendToCmdPose();

                std_msgs::Bool successMsg;
                successMsg.data = true;
                schunkServeDrinkDonePublisher_.publish(successMsg);

                m_cmdPose.position.x = -0.17378113231330397;
                m_cmdPose.position.y = 0.08158250100704804;
                m_cmdPose.position.z = 0.5473096569815927;
                m_cmdPose.orientation.x = -0.6497918897464642;
                m_cmdPose.orientation.y = 9.412819160463078e-05;
                m_cmdPose.orientation.z = 0.7601121445840632;
                m_cmdPose.orientation.w = 0.0001371693090430865;
                sendToCmdPose();

                m_leaveDrink = false;
                m_first = true;

            }




            //schunkOrderPublisher.publish(true);

            // TODO: Tell that oredr is recieved
            orderReciv = false;


        r.sleep();

}}
}

//rosrun fkie_master_discovery master_discovery __ns:="schunk"

//rosrun fkie_master_sync master_sync _sync_hosts:="["christmas_state_machine"]" __ns:="schunk"










