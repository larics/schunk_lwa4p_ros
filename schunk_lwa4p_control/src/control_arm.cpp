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

    if (startChristmas_){
        pickSugarService_ = nodeHandleWithoutNs_.advertiseService("/schunk_get_sugar", &ControlArm::schunkPickSugarServiceCallback, this);
        putSugarService_ = nodeHandleWithoutNs_.advertiseService("/schunk_put_sugar", &ControlArm::schunkPutSugarServiceCallback, this);
        returnSugarService_ = nodeHandleWithoutNs_.advertiseService("/schunk_return_sugar", &ControlArm::schunkReturnSugarServiceCallback, this);
        homingService_ = nodeHandleWithoutNs_.advertiseService("/schunk_go_home", &ControlArm::schunkHomingServiceCallback, this); 

        // Christmas publishers
        schunkAction1Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Bool>("/schunk_get_sugar_done", 1);
        schunkAction2Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Bool>("/schunk_put_sugar_done", 1);
        schunkAction3Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Bool>("/schunk_return_sugar_done", 1);
        schunkAction4Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Bool>("/schunk_go_home_done", 1);
    }

    //addCollisionObjectServiceClient_ = nodeHandle_.serviceClient<std_srvs::Trigger>("scene/add_collisions");
    ROS_INFO("[ControlArm] Initialized service clients. ");



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

    m_moveGroupPtr->setMaxVelocityScalingFactor(0.1);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(0.1);

    //TODO: Set start state :) as it could be correctly

    getCurrentArmState();

    // Update current joint values --> This should solve problem of deviates from current state
    m_moveGroupPtr->setStartStateToCurrentState();

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

    m_moveGroupPtr->setMaxVelocityScalingFactor(0.5);
    m_moveGroupPtr->setMaxAccelerationScalingFactor(0.1);

    //TODO: Set start state :) as it could be correctly

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

};

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

    // Add table in basement
    collisionObject1.id = "table";

    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.5;
    primitive.dimensions[1] = 1.0;
    primitive.dimensions[2] = 0.02;

    // A table pose (specified relative to frame_id)
    geometry_msgs::Pose table_pose;
    table_pose.orientation.w = 1.0;
    table_pose.position.x = -0.5;
    table_pose.position.y = 0.0;
    table_pose.position.z = 0.75;

    collisionObject1.primitives.push_back(primitive);
    collisionObject1.primitive_poses.push_back(table_pose);
    collisionObject1.operation = collisionObject1.ADD;

    collisionObjects.push_back(collisionObject1);

    collisionObject2.id = "wall";
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.1;
    primitive.dimensions[1] = 3.0;
    primitive.dimensions[2] = 2.0;

    geometry_msgs::Pose wall_pose;
    wall_pose.orientation.w = 1.0;
    wall_pose.position.x = -0.70;
    wall_pose.position.y = 0.0;
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
        primitive1.dimensions[2] = 1.16;

        geometry_msgs::Pose object_pose;
        object_pose.orientation.w = 1.0;
        object_pose.position.x = 0.0;
        object_pose.position.y = -0.6;
        object_pose.position.z = 0.58;
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

// TODO: Move this to utils.cpp
double ControlArm::VectorSize(geometry_msgs::Vector3 vector)
{
    // TODO: Add this to utils method
    return sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}
double ControlArm::DotProduct(geometry_msgs::Vector3 v_A, geometry_msgs::Vector3 v_B)
{
    // TODO: Move this to utils method
    return v_A.x * v_B.x + v_A.y * v_B.y + v_A.z * v_B.z;
}
geometry_msgs::Vector3 ControlArm::getClosestPointOnLine(geometry_msgs::Vector3 line_point,geometry_msgs::Vector3 line_vector, geometry_msgs::Vector3 point)
{
    // TODO: Add this to utils methods
    double x1 = line_point.x-point.x;
    double y1 = line_point.y-point.y;
    double z1 = line_point.z-point.z;
    double vx = line_vector.x;
    double vy = line_vector.x;
    double vz = line_vector.x;
    geometry_msgs::Vector3 p1, vector, result;
    p1.x = x1;
    p1.y = y1;
    p1.z = z1;
    // std::cout<<"transform base power_line "<<p1;
    vector.x = vx;
    vector.y = vy;
    vector.z = vz;
    // std::cout<<"line vector "<<vector;
    double t=-DotProduct(p1,vector)/VectorSize(vector)/VectorSize(p1);
    //std::cout<<"t "<<t<<std::endl;
    result.x = x1 + t * vx;
    result.y = y1 + t * vy;
    result.z = z1 + t * vz;
    return result;

}
float ControlArm::round(float var){

    float value = (int)(var * 1000 + .5);
    return (float) value/1000;

}

// ChristmasServices
bool ControlArm::schunkPickSugarServiceCallback(christmas_fair_common::StartTrajectorySrvRequest &req, christmas_fair_common::StartTrajectorySrvResponse &res){

    m_startSchunkPick = true;
    m_schunkPickId = req.id;

    return true;

}

bool ControlArm::schunkPutSugarServiceCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res){

    m_putSchunkSugar = true;

    return true;
}

bool ControlArm::schunkReturnSugarServiceCallback(christmas_fair_common::StartTrajectorySrvRequest &req, christmas_fair_common::StartTrajectorySrvResponse &res){

    m_returnSchunkSugar = true;
    m_returnPickId = req.id;


    return true;
}

bool ControlArm::schunkHomingServiceCallback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res){
    m_homingSchunk = true;

    res.success = true;

    return true;
}

void ControlArm::run() {

    ros::Rate r(25);
    bool executed = false;

    bool grasped_object = false;

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
        double timeout = 1; 
        bool successIK;

        successIK = getIK(attempts, timeout);

        bool magnetic_localization = false;
        try {
            if(magnetic_localization){
                // TODO: Add to specific method or class to handle this, maybe create ControlArm node as virtual
                // class and this could be MagneticArm which inherits that class
                tf::StampedTransform transform1; tf::StampedTransform transform2; tf::StampedTransform transform3;
                tf::StampedTransform powerline0_transform; tf::StampedTransform powerline1_transform;
                geometry_msgs::PoseStamped powerline0Pose; geometry_msgs::PoseStamped powerline1Pose;
                geometry_msgs::Quaternion poseQuaternion;

                std::string needed_frame = "base_link";
                std::string target_frame = "lwa4p_base_link";
                listener.lookupTransform(needed_frame, "power_line0_n", ros::Time(0), powerline0_transform);
                listener.lookupTransform(needed_frame, "power_line1_n", ros::Time(0), powerline1_transform);
                listener.lookupTransform(target_frame, "power_line0_n", ros::Time(0), transform1);
                listener.lookupTransform(target_frame, "power_line1_n", ros::Time(0), transform2);

                tf::Matrix3x3 rotation_matrix(transform1.getRotation()); //getBasis
                tf::Vector3 x_dir = rotation_matrix.getColumn(0);
                geometry_msgs::Vector3 x_dir_converted; geometry_msgs::Vector3 line1; geometry_msgs::Vector3 line2;
                geometry_msgs::Vector3 base; base.x = 0; base.y = 0; base.z = 0;
                x_dir_converted.x = x_dir.x(); x_dir_converted.y = x_dir.y(); x_dir_converted.z =  x_dir.z();
                line1.x = transform1.getOrigin().x(); line1.y = transform1.getOrigin().y(); line1.z = transform1.getOrigin().z();
                line2.x = transform2.getOrigin().x(); line2.y = transform2.getOrigin().y(); line2.z = transform2.getOrigin().z();
                geometry_msgs::Vector3 close1; geometry_msgs::Vector3 close2;
                close1 = getClosestPointOnLine(line1, x_dir_converted, base);
                close2 = getClosestPointOnLine(line2, x_dir_converted, base);

                // Transform from last link (ee frame) to the end of the separator main link of the separator
                listener.lookupTransform("lwa4p_link6", "separator_main", ros::Time(0), transform3);

                //ROS_INFO_STREAM("[world] x: " << (transform1.getOrigin().x() + transform2.getOrigin().x()) / 2);
                //ROS_INFO_STREAM("[world] y: " << transform1.getOrigin().y());
                //ROS_INFO_STREAM("[world] z: " << transform1.getOrigin().z());

                tf::Transform final_transform;
                tf::Vector3 final_translation;
                final_translation.setX((close1.x + close2.x)/2);
                final_translation.setY((close1.y + close2.y)/2);
                final_translation.setZ((close1.z + close2.z)/2 - transform3.getOrigin().z());
                final_transform.setRotation(transform1.getRotation());
                final_transform.setOrigin(final_translation);
                broadcaster.sendTransform(tf::StampedTransform(final_transform, ros::Time::now(), target_frame, "goal_frame"));

                powerline0Pose.pose.position.x = powerline0_transform.getOrigin().x();
                powerline0Pose.pose.position.y = powerline0_transform.getOrigin().y();
                powerline0Pose.pose.position.z = powerline0_transform.getOrigin().z();

                powerline0Pose.pose.orientation.x = powerline0_transform.getRotation().x();
                powerline0Pose.pose.orientation.y = powerline0_transform.getRotation().y();
                powerline0Pose.pose.orientation.z = powerline0_transform.getRotation().z();
                powerline0Pose.pose.orientation.w = powerline0_transform.getRotation().w();

                powerline1Pose.pose.position.x = powerline1_transform.getOrigin().x();
                powerline1Pose.pose.position.y = powerline1_transform.getOrigin().y();
                powerline1Pose.pose.position.z = powerline1_transform.getOrigin().z();

                powerline1Pose.pose.orientation.x = powerline1_transform.getRotation().x();
                powerline1Pose.pose.orientation.y = powerline1_transform.getRotation().y();
                powerline1Pose.pose.orientation.z = powerline1_transform.getRotation().z();
                powerline1Pose.pose.orientation.w = powerline1_transform.getRotation().w();

                powerline0Pose.header.stamp = ros::Time::now();
                powerline0PosePublisher.publish(powerline0Pose);

                powerline1Pose.header.stamp = ros::Time::now();
                powerline1PosePublisher.publish(powerline1Pose);

            }
        }catch(const std::exception& e){
                ROS_DEBUG_STREAM("Failed...");
        }

        //Eigen::MatrixXd m_; 
        //Eigen::Vector3d testVector(0.0, 0.0, 0.0);
        //m_ = getJacobian(testVector);

        if (startChristmas_){

            wsg_50_common::Move moveSrv;
            wsg_50_common::Conf setForceSrv;
            wsg_50_common::Move graspSrv;

                // WORKS OK!
                // TODO: Fix service type (enable integer passing to choose pick place)
                // TODO: Check pick place for gripper
                if(m_startSchunkPick){

                    double tolerance = m_moveGroupPtr->getGoalOrientationTolerance();
                    //ROS_INFO_STREAM("Current orientation tolerance is:" << tolerance);
                    double position_tolerance = m_moveGroupPtr->getGoalPositionTolerance();
                    //ROS_INFO_STREAM("Current position tolerance is: " << position_tolerance);

                    //m_moveGroupPtr->setGoalOrientationTolerance(0.3);
                    //m_moveGroupPtr->setGoalPositionTolerance(0.1);
                    //m_moveGroupPtr->setGoalJointTolerance(0.2);
                    //m_moveGroupPtr->setPlanningTime(10);
                    //m_moveGroupPtr->setNumPlanningAttempts(20);
                    std::string PlanningFrame = m_moveGroupPtr->getPlanningFrame().c_str();
                    //ROS_INFO_STREAM("Current planning frame is: " << PlanningFrame);

                    moveSrv.request.width = 65;
                    moveSrv.request.speed = 25;
                    gripperMoveServiceClient_.call(moveSrv.request,
                                                   moveSrv.response);


                    m_cmdJoint.position = {-2.4378235393081193, 0.48694686130641796, 1.6780468093299483, 1.297687205442824, 0.9831963275259656, -1.0706198697583615};
                    setCmdJoint();
                    sendToCmdJoint();


                    // FIRST CUP
                    if(m_schunkPickId == 0){

                        // pregrasp pose
                        //m_cmdJoint.position = {-2.521337544016048, 0.10229374745938764, 1.4690087248185872, 1.4239966834096536, 0.9746965740687532, -1.1745018668370641};
                        m_cmdJoint.position = {-2.008001304419476, 0.41318924711713756, 1.8207798355580445, 1.232429344710756, 0.47485172959009725, -1.160748672331349};
                        setCmdJoint();
                        sendToCmdJoint();

                        //m_cmdJoint.position = {-2.521337544016048, 0.10229374745938764, 1.4690436314036273, 1.4239443235320937, 0.9746267608986734, -1.1744146003744644};
                        m_cmdJoint.position = {-2.1549580274373983, 0.2767568594887408, 1.8709056916753215, 1.5959290680236147, 0.5924520145894752, -1.568806651447623};
                        setCmdJoint();
                        sendToCmdJoint();

                        // aDD 3RD POINT

                        //m_cmdJoint.position = {-2.430789862422582, 0.04305727264670011, 1.4006441780179693, 1.387466942165412, 0.8886343886529129, -1.1449534426008};
                        m_cmdJoint.position = {-2.045595696507434, 0.15440927892393833, 1.7794155322857788, 1.6622515795993993, 0.4852538919319834, -1.6409760160175886};
                        setCmdJoint();
                        sendToCmdJoint();

                        //m_cmdJoint.position = {-2.3728623845488905, 8.726646259971647e-05, 1.3590704352354643, 1.3671862162572381, 0.832888572344214, -1.132352165401401};
                        m_cmdJoint.position = {-1.9965519445263933, 0.07946484084330183, 1.7149081631320684, 1.6964600329384885, 0.4374667770123787, -1.6760920405677147};
                        setCmdJoint();
                        sendToCmdJoint();



                        //position: [-0.2339439329373199, 1.1447265497980408, 1.9574740225742402, -1.3984625164529765, 1.3990733816911745, 0.8330980118544533]

                        // position: [-0.2339439329373199, 1.1447265497980408, 1.9574740225742402, -1.3984625164529765, 1.3990733816911745, 0.8330980118544533]

                        // position: [-0.2339439329373199, 1.1447265497980408, 1.9574740225742402, -1.3984625164529765, 1.3990733816911745, 0.8330980118544533]

                        // position: [-0.2339439329373199, 1.1447265497980408, 1.9574740225742402, -1.3984625164529765, 1.3990733816911745, 0.8330980118544533]

                        // position: [-1.7381559487686327, 0.4680623987998393, 1.952622007253696, 1.0925935650409702, 0.1977807108359974, -0.9645038512371064]



                    }

                    // SECOND CUP
                    if (m_schunkPickId == 1){
                        m_cmdJoint.position = {-1.2651891747706896, 0.3448421536090396, 1.9117289428794688, -1.5386473619731609, 0.29764845063511297, 1.575020023584723};
                        setCmdJoint();
                        sendToCmdJoint();

                        m_cmdJoint.position = {-1.3429784995320768, 0.1355248164173597, 1.7610197619697587, -1.7871473408721135, 0.225007847167109, 1.829873000960935};
                        setCmdJoint();
                        sendToCmdJoint();

                        // GRASP:

                        // AFTERGRASP:
                        //[]

                    }




                    setForceSrv.request.val = 20;

                    gripperSetForceServiceClient_.call(setForceSrv.request,
                                                       setForceSrv.response);

                    // PICK SUGAR
                    //sendToCmdPoses(objectPoses);


                    ROS_INFO_STREAM("Grasping cup! ");

                    graspSrv.request.width = 30;
                    graspSrv.request.speed = 10;
                    gripperGraspServiceClient_.call(graspSrv.request,
                                                    graspSrv.response);
                    ROS_INFO_STREAM("Grasp response is: " << graspSrv.response);

                    m_startSchunkPick = false;
                    // TODO: Put topic that declares end for each action
                    std_msgs::Bool action1;
                    if(graspSrv.response.error == 0)
                    {

                        if (m_schunkPickId == 0){
                            //[]
                            // uper pose
                            m_cmdJoint.position = {-2.0279678710622915, -0.02345722514680379, 1.0886316676389431, 0.8422435371349036, 0.6452482244623036, -0.6971368631240951};
                            setCmdJoint();
                            sendToCmdJoint();

                            m_cmdJoint.position = {-0.9776112739195838, -0.07499679795819635, 1.0752973521537061, -0.2780833097202565, 0.42339942324130436, 0.28763026072866554};
                            //m_cmdJoint.position = {-2.394801173246459, -0.0797790001086608, 1.065540961635058, 1.2060748730056414, 0.9133482508611526, -0.8721410272215665};
                            setCmdJoint();
                            sendToCmdJoint();

                            // TODO: Dodati i ovo za aftergrasp pose
                            //position: [-2.4059363738741832, -0.14360569085409344, 0.87416560915388, 1.1288440536048925, 0.972305472993521, -0.7320958080415415]

                        }

                        if (m_schunkPickId == 1){

                            m_cmdJoint.position = {-1.3579185179291482, 0.10571459279329654, 1.406962269910189, -0.6526833270757995, 0.34159584120033015, 0.6613750667507312};
                            setCmdJoint();
                            sendToCmdJoint();
                        }

                        //TODO: Add for cup 2



                        action1.data = true;
                        schunkAction1Publisher.publish(action1);
                    }
                    else{
                        // TODO: Add after grasp pose
                        action1.data = false;
                        schunkAction1Publisher.publish(action1);
                    }

                }

                // TODO: Check orientation constraints
                if(m_putSchunkSugar){


                    grasped_object = true;

                    // Object 0
                    // position: [-2.3892335729325977, -0.05419247327442393, 1.150555949499702, 1.246845764332229, 0.888337682680074, -0.9412735188930619]

                    //position: [-2.4197593815499787, -0.27689648582890036, 0.5253266515577732, 1.0253983888391887, 1.0845825037743162, -0.516216032862363]

                    // SUGAR MOVEMENT
                    m_cmdJoint.position = {-0.0007330382858376184, 0.3463605900582747, 1.329417291244081, 0.16505578736110374, 0.5575803361346285, -0.05927138139772743};
                    setCmdJoint();
                    sendToCmdJoint();

                    m_cmdJoint.position = {0.09752899860144312, 0.4864930757008994, 1.9822227913675199, -0.2795668395844517, 0.040578905108868156, 0.3568849254478005};
                    setCmdJoint();
                    sendToCmdJoint();


                    setForceSrv.request.val = 80;

                    gripperSetForceServiceClient_.call(setForceSrv.request,
                                                       setForceSrv.response);


                    ROS_INFO_STREAM("Sugaring 1");

                    graspSrv.request.width = 20;
                    graspSrv.request.speed = 50;
                    gripperGraspServiceClient_.call(graspSrv.request,
                                                    graspSrv.response);

                    ROS_INFO_STREAM("Service response: " << graspSrv.response);

                    ros::Duration(1.0).sleep();
                    ROS_INFO_STREAM("============================");
                    ROS_INFO_STREAM("Releasing 1");
                    moveSrv.request.width = 40;
                    moveSrv.request.speed = 50;
                    gripperReleaseServiceClient_.call(moveSrv.request,
                                                      moveSrv.response);

                    ROS_INFO_STREAM("Service response: " << moveSrv.response);

                    ros::Duration(1.0).sleep();

                    ROS_INFO_STREAM("============================");
                    ROS_INFO_STREAM("Sugaring 2");
                    graspSrv.request.width = 20;
                    graspSrv.request.speed = 50;
                    gripperGraspServiceClient_.call(graspSrv.request,
                                                    graspSrv.response);

                    ROS_INFO_STREAM("Service response: " << graspSrv.response);

                    ros::Duration(1.0).sleep();

                    ROS_INFO_STREAM("============================");
                    ROS_INFO_STREAM("Releasing 2");
                    moveSrv.request.width = 40;
                    moveSrv.request.speed = 50;
                    gripperReleaseServiceClient_.call(moveSrv.request,
                                                      moveSrv.response);

                    ROS_INFO_STREAM("Service response: " << moveSrv.response);

                    // TODO: Put topic that declares end for each action
                    m_putSchunkSugar = false;
                    std_msgs::Bool action2;
                    action2.data = true;
                    schunkAction2Publisher.publish(action2);
                }

                // TODO: Check pose orientation constraints
                if(m_returnSchunkSugar){

                    ROS_INFO_STREAM("Returning sugar!");
                    //ros::Duration(1.0).sleep();

                    // Return first cup
                    if (m_returnPickId == 0){

                        //m_cmdJoint.position = {-1.133853148558116, -0.09463175204313255, 0.6761580055151232, 0.07298966931840287, 0.8074242185576168, 0.0999375529691953};
                        // SUGAR MOVEMENT
                        m_cmdJoint.position = {-2.0279678710622915, -0.02345722514680379, 1.0886316676389431, 0.8422435371349036, 0.6452482244623036, -0.6971368631240951};
                        setCmdJoint();
                        sendToCmdJoint();

                        m_cmdJoint.position = {-1.9965519445263933, 0.07946484084330183, 1.7149081631320684, 1.6964600329384885, 0.4374667770123787, -1.6760920405677147};
                        setCmdJoint();
                        sendToCmdJoint();



                        moveSrv.request.width = 65;
                        moveSrv.request.speed = 10;
                        gripperReleaseServiceClient_.call(moveSrv.request,
                                                          moveSrv.response);

                        ros::Duration(1.0).sleep();

                        //m_cmdJoint.position = {-2.430789862422582, 0.04305727264670011, 1.4006441780179693, 1.387466942165412, 0.8886343886529129, -1.1449534426008};
                        m_cmdJoint.position = {-2.045595696507434, 0.15440927892393833, 1.7794155322857788, 1.6622515795993993, 0.4852538919319834, -1.6409760160175886};
                        setCmdJoint();
                        sendToCmdJoint();

                        m_cmdJoint.position = {-2.2831350077038617, 0.37273251505590904, 1.931119550869126, 1.548491018954409, 0.7207686211960983, -1.5099192424853343};
                        setCmdJoint();
                        sendToCmdJoint();




                    }
                    // Return second cup
                    if (m_returnPickId == 1){

                        m_cmdJoint.position = {-1.3579185179291482, 0.10571459279329654, 1.406962269910189, -0.6526833270757995, 0.34159584120033015, 0.6613750667507312};
                        setCmdJoint();
                        sendToCmdJoint();
                        // ADD for second pose

                        m_cmdJoint.position = {-1.386000865593737, 0.14266321305801652, 1.753497392893663, -1.762433478663874, 0.18018779197589457, 1.802453878412104};
                        setCmdJoint();
                        sendToCmdJoint();

                        m_cmdJoint.position =  {-1.3429610462395567, 0.13559462958743948, 1.7610895751398383, -1.7871997007496736, 0.2250602070446688, 1.8299079075459748};
                        setCmdJoint();
                        sendToCmdJoint();

                        moveSrv.request.width = 65;
                        moveSrv.request.speed = 10;
                        gripperReleaseServiceClient_.call(moveSrv.request,
                                                          moveSrv.response);

                        ros::Duration(1.0).sleep();

                        m_cmdJoint.position = {-1.149665831581185, 0.48832567141549343, 1.965921416153893, -1.3643762361615273, 0.4334525197327917, 1.4450279008961853};
                        setCmdJoint();
                        sendToCmdJoint();


                    }



                    ROS_INFO_STREAM("============================");
                    ROS_INFO_STREAM("Releasing");




                    m_returnSchunkSugar = false;
                    std_msgs::Bool action3;
                    action3.data = true;
                    schunkAction3Publisher.publish(action3);



                }

                if(m_homingSchunk){
                    m_cmdJoint.position = {-1.63254, 0.610812, 1.638340, 0.134303, 0.552536, -0.07894124}   ;
                    setCmdJoint();
                    sendToCmdJoint();
                    
                    m_homingSchunk = false;
                    std_msgs::Bool action4; 
                    action4.data = true;
                    schunkAction4Publisher.publish(action4); 
                }

        r.sleep(); 
    }

}
}








