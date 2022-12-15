#include "control_arm/control_arm.h"

ControlArm::ControlArm(ros::NodeHandle nh) : nodeHandle_(nh) {

  ROS_INFO("[ControlArm] Node started.");

  // Read parameters from config file.
  if (!readParameters()) {
    ros::requestShutdown();
  }

  // Initial sleep (waiting for move group and rest of the MoveIt stuff to
  // initialize.)
  usleep(sleepMs_);

  // Initialize class
  init();

  // Find out basic info
  getBasicInfo();
}

ControlArm::~ControlArm() {}

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
  int currentJointCmdQueueSize;

  std::string disableCollisionServiceName;
  std::string addCollisionObjectServiceName;
  std::string startPositionControllersServiceName;
  std::string startJointTrajectoryControllerServiceName;
  std::string startJointGroupPositionControllerServiceName;
  std::string startJointGroupVelocityControllerServiceName;
  std::string sendArmToHomingPoseServiceName;
  std::string getIkServiceName;

  nodeHandle_.param("publishers/display_trajectory_topic",
                    displayTrajectoryTopicName,
                    std::string("move_group/display_planned_path"));
  nodeHandle_.param("publishers/queue_size", displayTrajectoryQueueSize, 1);
  nodeHandle_.param("publishers/current_pose", currentPoseTopicName,
                    std::string("tool/current_pose"));
  nodeHandle_.param("publishers/queue_size", currentPoseTopicQueueSize, 1);

  nodeHandle_.param("subscribers/cmd_pose_topic", cmdPoseTopicName,
                    std::string("arm/command/pose"));
  nodeHandle_.param("subscribers/queue_size", cmdPoseTopicQueueSize, 1);
  nodeHandle_.param("subscribers/cmd_tool_orientation_topic",
                    cmdToolOrientationTopicName,
                    std::string("tool/command/orientation"));
  nodeHandle_.param("subscribers/queue_size", cmdToolOrientationTopicQueueSize,
                    1);
  nodeHandle_.param("subscribers/cmd_delta_pose_topic", cmdDeltaPoseTopicName,
                    std::string("arm/command/delta_pose"));
  nodeHandle_.param("subscribers/queue_size", cmdDeltaPoseTopicQueueSize, 1);
  nodeHandle_.param("services/disable_collision_service",
                    disableCollisionServiceName,
                    std::string("tool/disable_collision"));
  nodeHandle_.param("services/add_collision_object",
                    addCollisionObjectServiceName,
                    std::string("scene/add_collisions"));
  nodeHandle_.param("services/start_position_controllers",
                    startPositionControllersServiceName,
                    std::string("controllers/start_position_controllers"));
  nodeHandle_.param(
      "services/start_joint_trajectory_controller",
      startJointTrajectoryControllerServiceName,
      std::string("controllers/start_joint_trajectory_controller"));
  nodeHandle_.param(
      "services/start_joint_group_position_controller",
      startJointGroupPositionControllerServiceName,
      std::string("controllers/start_joint_group_position_controller"));
  nodeHandle_.param(
      "services/start_joint_group_velocity_controller",
      startJointGroupVelocityControllerServiceName,
      std::string("controllers/start_joint_group_velocity_controller"));
  nodeHandle_.param("services/send_arm_to_homing_pose",
                    sendArmToHomingPoseServiceName,
                    std::string("arm/send_arm_to_homing_pose"));
  nodeHandle_.param("services/get_ik",
                    getIkServiceName,
                    std::string("get_ik"));

  ROS_INFO("[ControlArm] Initializing subscribers/publishers...");
  displayTrajectoryPublisher_ =
      nodeHandle_.advertise<moveit_msgs::DisplayTrajectory>(
          displayTrajectoryTopicName, displayTrajectoryQueueSize);
  currentPosePublisher_ = nodeHandle_.advertise<geometry_msgs::Pose>(
      currentPoseTopicName, currentPoseTopicQueueSize);
  cmdJoint1Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Float64>(
      std::string("lwa4p/joint_1_position_controller/command"), 1);
  cmdJoint2Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Float64>(
      std::string("lwa4p/joint_2_position_controller/command"), 1);
  cmdJoint3Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Float64>(
      std::string("lwa4p/joint_3_position_controller/command"), 1);
  cmdJoint4Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Float64>(
      std::string("lwa4p/joint_4_position_controller/command"), 1);
  cmdJoint5Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Float64>(
      std::string("lwa4p/joint_5_position_controller/command"), 1);
  cmdJoint6Publisher = nodeHandleWithoutNs_.advertise<std_msgs::Float64>(
      std::string("lwa4p/joint_6_position_controller/command"), 1);
  cmdJointGroupPositionPublisher =
      nodeHandleWithoutNs_.advertise<std_msgs::Float64MultiArray>(
          std::string("lwa4p/joint_group_position_controller/command"), 1);
  cmdJointGroupVelocityPublisher =
      nodeHandleWithoutNs_.advertise<std_msgs::Float64MultiArray>(
          std::string("lwa4p/joint_group_velocity_controller/command"), 1);

  armCmdPoseSubscriber_ = nodeHandle_.subscribe<geometry_msgs::Pose>(
      cmdPoseTopicName, cmdPoseTopicQueueSize, &ControlArm::cmdPoseCallback,
      this);
  armCmdToolOrientationSubscriber_ =
      nodeHandle_.subscribe<geometry_msgs::Point>(
          cmdToolOrientationTopicName, cmdToolOrientationTopicQueueSize,
          &ControlArm::cmdToolOrientationCallback, this);
  armCmdDeltaPoseSubscriber_ = nodeHandle_.subscribe<geometry_msgs::Pose>(
      cmdDeltaPoseTopicName, cmdDeltaPoseTopicQueueSize,
      &ControlArm::cmdDeltaPoseCallback, this);
  ROS_INFO("[ControlArm] Initialized subscribers/publishers.");

  // Initialize Services
  ROS_INFO("[ControlArm] Initializing services...");
  disableCollisionService_ = nodeHandle_.advertiseService(
      disableCollisionServiceName, &ControlArm::disableCollisionServiceCallback,
      this);
  addCollisionObjectService_ = nodeHandle_.advertiseService(
      addCollisionObjectServiceName,
      &ControlArm::addCollisionObjectServiceCallback, this);
  startPositionControllersService_ =
      nodeHandle_.advertiseService(startPositionControllersServiceName,
                                   &ControlArm::startPositionControllers, this);
  getIkService_ = nodeHandle_.advertiseService(getIkServiceName, &ControlArm::getIkServiceCallback, this);

  startJointTrajectoryControllerService_ = nodeHandle_.advertiseService(
      startJointTrajectoryControllerServiceName,
      &ControlArm::startJointTrajectoryController, this);
  startJointGroupPositionControllerService_ = nodeHandle_.advertiseService(
      startJointGroupPositionControllerServiceName,
      &ControlArm::startJointGroupPositionController, this);
  startJointGroupVelocityControllerService_ = nodeHandle_.advertiseService(
      startJointGroupVelocityControllerServiceName,
      &ControlArm::startJointGroupVelocityController, this);
  sendArmToHomingPoseService_ = nodeHandle_.advertiseService(
      sendArmToHomingPoseServiceName, &ControlArm::sendArmToHomingPose, this);
  // executeCartesianPathService_ =
  // nodeHandle_.advertiseService(executeCartesianPathServiceName,
  // &ControlArm::executeCartesianServiceCallback, this);
  ROS_INFO("[ControlArm] Initialized services.");

  // Initialize Clients for other services
  ROS_INFO("[ControlArm] Initializing service clients...");
  applyPlanningSceneServiceClient_ =
      nodeHandleWithoutNs_.serviceClient<moveit_msgs::ApplyPlanningScene>(
          "apply_planning_scene");
  applyPlanningSceneServiceClient_.waitForExistence();
  switchControllerServiceClient_ =
      nodeHandleWithoutNs_
          .serviceClient<controller_manager_msgs::SwitchController>(
              "lwa4p/controller_manager/switch_controller");
  switchControllerServiceClient_.waitForExistence();
  listControllersServiceClient_ =
      nodeHandleWithoutNs_
          .serviceClient<controller_manager_msgs::ListControllers>(
              "lwa4p/controller_manager/list_controllers");
  listControllersServiceClient_.waitForExistence();
  switchToPositionControllerServiceClient_ =
      nodeHandle_.serviceClient<std_srvs::Trigger>(
          startPositionControllersServiceName);
  switchToTrajectoryControllerServiceClient_ =
      nodeHandle_.serviceClient<std_srvs::Trigger>(
          startJointTrajectoryControllerServiceName);

  // addCollisionObjectServiceClient_ =
  // nodeHandle_.serviceClient<std_srvs::Trigger>("scene/add_collisions");
  ROS_INFO("[ControlArm] Initialized service clients. ");
}

bool ControlArm::setMoveGroup() {

  ROS_INFO("[ControlArm] Setting move group.");

  // MoveIt move group
  static const std::string groupName = "arm";
  m_moveGroupPtr =
      new moveit::planning_interface::MoveGroupInterface(groupName);

  // Allow replanning
  m_moveGroupPtr->allowReplanning(true);

  // Get current robot arm state
  getCurrentArmState();

  return true;
}

bool ControlArm::setPlanningScene() {

  ROS_INFO("[ControlArm] Setting planning scene.");

  // MoveIt planning scene setup as seen
  // (http://docs.ros.org/en/melodic/api/moveit_tutorials/html/doc/planning_scene/planning_scene_tutorial.html)
  robot_model_loader::RobotModelLoader m_robotLoader("robot_description");
  robot_model::RobotModelPtr kinematic_model = m_robotLoader.getModel();
  m_planningScenePtr = new planning_scene::PlanningScene(kinematic_model);
  ROS_INFO("[ControlArm] Model frame: %s",
           kinematic_model->getModelFrame().c_str());
  return true;
}

// This is wrong, we should pass cmdPose as argument into function and then set
// it if we plan to use setters
bool ControlArm::setCmdPose() {

  if (moveGroupInitialized_) {

    m_moveGroupPtr->setPoseTarget(m_cmdPose);
    return true;
  }
  return false;
}

void ControlArm::getBasicInfo() {

  if (moveGroupInitialized_) {

    ROS_INFO("[ControlArm] Reference planning frame: %s",
             m_moveGroupPtr->getPlanningFrame().c_str());
    ROS_INFO("[ControlArm] Reference end effector frame: %s",
             m_moveGroupPtr->getEndEffectorLink().c_str());
  }
}

void ControlArm::cmdPoseCallback(const geometry_msgs::Pose::ConstPtr &msg) {

  ROS_INFO("[ControlArm] Recieved cmd_pose...");

  // Set CMD pose
  m_cmdPose.position = msg->position;
  m_cmdPose.orientation = msg->orientation;

  sendToCmdPose();
}

void ControlArm::cmdDeltaPoseCallback(
    const geometry_msgs::Pose::ConstPtr &msg) {

  ROS_INFO("[ControlArm] Recieved cmd_delta_pose...");

  // Set delta CMD pose
  m_cmdDeltaPose.position = msg->position;
  m_cmdDeltaPose.orientation = msg->orientation;

  sendToDeltaCmdPose();
}

void ControlArm::cmdToolOrientationCallback(
    const geometry_msgs::Point::ConstPtr &msg) {

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

bool ControlArm::sendToCmdPose() {

  setCmdPose();

  // Call planner, compute plan and visualize it
  moveit::planning_interface::MoveGroupInterface::Plan plannedPath;

  // plan Path
  bool success = (m_moveGroupPtr->plan(plannedPath) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO("[ControlArm] Visualizing plan 1 (pose goal) %s",
           success ? "" : "FAILED");

  // Requires async spinner to work (Added asyncMove/non-blocking)
  if (success) {
    if (blockingMovement) {
      m_moveGroupPtr->move();
    } else {
      m_moveGroupPtr->asyncMove();
    }
  }

  return success;
}

void ControlArm::sendToCmdPoses(std::vector<geometry_msgs::Pose> poses) {
  for (int i; i < poses.size(); ++i) {
    ROS_INFO_STREAM("[ControlArmNode] Visiting pose " << i);
    m_cmdPose.position = poses.at(i).position;
    m_cmdPose.orientation = poses.at(i).orientation;
    sendToCmdPose();
    // if (m_moveGroupPtr->state)
  }
}

bool ControlArm::sendToDeltaCmdPose() {

  // populate m_cmd pose
  Eigen::Affine3d currentPose_ =
      m_moveGroupPtr->getCurrentState()->getFrameTransform(
          "lwa4p_link6"); // Currently lwa4p_link6, possible to use end effector
                          // link
  geometry_msgs::Pose currentROSPose_;
  tf::poseEigenToMsg(currentPose_, currentROSPose_);

  ROS_INFO_STREAM("[ControlArm] currentROSPose_:" << currentROSPose_);

  geometry_msgs::Pose cmdPose;
  cmdPose.position.x = currentROSPose_.position.x + m_cmdDeltaPose.position.x;
  cmdPose.position.y = currentROSPose_.position.y + m_cmdDeltaPose.position.y;
  cmdPose.position.z = currentROSPose_.position.z + m_cmdDeltaPose.position.z;
  cmdPose.orientation.x =
      currentROSPose_.orientation.x + m_cmdDeltaPose.orientation.x;
  cmdPose.orientation.y =
      currentROSPose_.orientation.y + m_cmdDeltaPose.orientation.y;
  cmdPose.orientation.z =
      currentROSPose_.orientation.z + m_cmdDeltaPose.orientation.z;
  cmdPose.orientation.w =
      currentROSPose_.orientation.w + m_cmdDeltaPose.orientation.w;

  ROS_INFO_STREAM("[ControlArm] currentPose: " << cmdPose);

  // set CMD pose
  m_cmdPose = cmdPose;

  sendToCmdPose();

  return true;
}

void ControlArm::addCollisionObject(moveit_msgs::PlanningScene &planningScene) {

  ROS_INFO("Adding collision object...");

  std::vector<moveit_msgs::CollisionObject> collisionObjects;
  moveit_msgs::CollisionObject collisionObject1;
  moveit_msgs::CollisionObject collisionObject2;
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

  for (std::size_t i = 0; i < collisionObjects.size(); ++i) {
    planningScene.world.collision_objects.push_back(collisionObjects.at(i));
  };

  ROS_INFO("Added collisions");
}

bool ControlArm::disableCollisionServiceCallback(
    std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {
  // TODO: Move this to specific script because it's related to magnetic
  // localization

  if (planningSceneInitialized_) {
    collision_detection::AllowedCollisionMatrix acm =
        m_planningScenePtr->getAllowedCollisionMatrix();

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
    // m_planningScenePtr->setPlanningSceneMsg(planningScene); --> Setting it
    // over mPlanningScenePtr;

    moveit_msgs::ApplyPlanningScene srv;
    srv.request.scene = planningScene;
    applyPlanningSceneServiceClient_.call(srv);

    bool debugOut = false;
    if (debugOut) {
      acm.print(std::cout);
      ROS_INFO("[ControlArm] Disabled collisions: %d",
               (bool)srv.response.success);
      collision_detection::AllowedCollisionMatrix acm_after =
          m_planningScenePtr->getAllowedCollisionMatrix();
      acm_after.print(std::cout);
    }
    return true;
  } else {
    return false;
  }
}

bool ControlArm::getIkServiceCallback(schunk_lwa4p_control::getIkRequest &req,
                                      schunk_lwa4p_control::getIkResponse &res)
{
    int attempts = 10;
    int timeout = 1;
    bool success = getIK(req.wantedPose, attempts, timeout);
    std::vector<double> jointPositions;
    m_currentRobotStatePtr->copyJointGroupPositions(m_jointModelGroupPtr,
                                                    jointPositions);

    sensor_msgs::JointState jointState;
    jointState.position = jointPositions;
    res.jointState = jointState;

    return success;

}

void ControlArm::getRunningControllers(
    std::vector<std::string> &runningControllerNames) {
  ROS_INFO("[ControlArm] Listing controllers: ");
  controller_manager_msgs::ListControllersRequest listReq;
  controller_manager_msgs::ListControllersResponse listRes;
  listControllersServiceClient_.call(listReq, listRes);
  // ROS_INFO_STREAM("[ControlArm] Controllers: " << listRes);

  for (std::size_t i = 0; i < listRes.controller.size(); ++i) {
    if (listRes.controller[i].state == "running") {
      // Additional constraints for controllers that must be active all of the
      // time
      if (listRes.controller[i].name != "joint_state_controller" &&
          listRes.controller[i].name != "distancer_right_position_controller" &&
          listRes.controller[i].name != "distancer_left_position_controller") {
        runningControllerNames.push_back(listRes.controller[i].name);

        ROS_INFO_STREAM(
            "[ControlArm] Stopping controller: " << listRes.controller[i].name);
      }
    }
  }

  // ROS_INFO_STREAM("[ControlArm] Running controllers are: " <<
  // runningControllerNames);
}

bool ControlArm::startJointTrajectoryController(
    std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

  std::vector<std::string> runningControllers;
  getRunningControllers(runningControllers);

  ROS_INFO("[ControlArm] Starting JointTrajectoryController...");
  // Stop running controllers
  controller_manager_msgs::SwitchControllerRequest switchControllerRequest;
  controller_manager_msgs::SwitchControllerResponse switchControllerResponse;
  for (std::size_t i = 0; i < runningControllers.size(); ++i) {
    switchControllerRequest.stop_controllers.push_back(runningControllers[i]);
  }
  switchControllerRequest.start_controllers.push_back(
      std::string("arm_controller"));
  switchControllerRequest.start_asap = true;
  // ontroller Manager: To switch controllers you need to specify a strictness
  // level of
  //  controller_manager_msgs::SwitchController::STRICT (2) or ::BEST_EFFORT
  //  (1). Defaulting to ::BEST_EFFORT.
  switchControllerRequest.strictness = 2;
  switchControllerRequest.timeout = 10;

  switchControllerServiceClient_.call(switchControllerRequest,
                                      switchControllerResponse);

  return switchControllerResponse.ok;
}

bool ControlArm::startJointGroupPositionController(
    std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

  std::vector<std::string> runningControllers;
  getRunningControllers(runningControllers);

  ROS_INFO("[ControlArm] Starting JointGroupPositionController...");
  controller_manager_msgs::SwitchControllerRequest switchControllerRequest;
  controller_manager_msgs::SwitchControllerResponse switchControllerResponse;
  // Stop running controllers
  for (std::size_t i = 0; i < runningControllers.size(); ++i) {
    switchControllerRequest.stop_controllers.push_back(runningControllers[i]);
  }
  switchControllerRequest.start_controllers.push_back(
      std::string("joint_group_position_controller"));
  switchControllerRequest.start_asap = true;
  switchControllerRequest.strictness = 2;
  switchControllerRequest.timeout = 10;

  switchControllerServiceClient_.call(switchControllerRequest,
                                      switchControllerResponse);
  ros::Duration(0.5).sleep();
  // TODO: Add enabling stuff for different controller type
  ROS_INFO("Sending all joints to zero");

  // activateJoints();
  sendZeros("group"); // Enables sending of commands to jointGroupController
  ros::Duration(0.5).sleep();
  // TODO: Add method for sending current joint states

  return switchControllerResponse.ok;
}

bool ControlArm::startJointGroupVelocityController(
    std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

  std::vector<std::string> runningControllers;
  getRunningControllers(runningControllers);

  ROS_INFO("[ControlArm] Start JointGroupVelocityController...");
  controller_manager_msgs::SwitchControllerRequest switchControllerRequest;
  controller_manager_msgs::SwitchControllerResponse switchControllerResponse;
  // Stop running controllers
  for (std::size_t i = 0; i < runningControllers.size(); ++i) {
    switchControllerRequest.stop_controllers.push_back(runningControllers[i]);
  }
  switchControllerRequest.start_controllers.push_back(
      std::string("joint_group_velocity_controller"));
  switchControllerRequest.start_asap = true;
  switchControllerRequest.strictness = 2;
  switchControllerRequest.timeout = 10;

  switchControllerServiceClient_.call(switchControllerRequest,
                                      switchControllerResponse);

  ROS_INFO("Switched to velocity controller");

  return switchControllerResponse.ok;
}

bool ControlArm::startPositionControllers(std_srvs::TriggerRequest &req,
                                          std_srvs::TriggerResponse &res) {

  std::vector<std::string> runningControllers;
  getRunningControllers(runningControllers);

  ROS_INFO("[ControlArm] Starting JointPosition controllers...");
  controller_manager_msgs::SwitchControllerRequest switchControllerRequest;
  controller_manager_msgs::SwitchControllerResponse switchControllerResponse;
  // Stop running controllers
  for (std::size_t i = 0; i < runningControllers.size(); ++i) {
    switchControllerRequest.stop_controllers.push_back(runningControllers[i]);
  }
  switchControllerRequest.start_controllers.push_back(
      std::string("joint_1_position_controller"));
  switchControllerRequest.start_controllers.push_back(
      std::string("joint_2_position_controller"));
  switchControllerRequest.start_controllers.push_back(
      std::string("joint_3_position_controller"));
  switchControllerRequest.start_controllers.push_back(
      std::string("joint_4_position_controller"));
  switchControllerRequest.start_controllers.push_back(
      std::string("joint_5_position_controller"));
  switchControllerRequest.start_controllers.push_back(
      std::string("joint_6_position_controller"));
  switchControllerRequest.start_asap = true;
  // Controller Manager: To switch controllers you need to specify a strictness
  // level of controller_manager_msgs::SwitchController::STRICT (2) or
  // ::BEST_EFFORT (1). Defaulting to ::BEST_EFFORT.
  switchControllerRequest.strictness = 2;
  switchControllerRequest.timeout = 10;

  switchControllerServiceClient_.call(switchControllerRequest,
                                      switchControllerResponse);
  ros::Duration(0.1).sleep();

  // sendZeros("position");

  return switchControllerResponse.ok;
}

bool ControlArm::addCollisionObjectServiceCallback(
    std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res) {

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

bool ControlArm::sendArmToHomingPose(std_srvs::TriggerRequest &req,
                                     std_srvs::TriggerResponse &res) {

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
  // Switch to position controller and send commands for each joint to reach 0
  // configuration of arm
  ros::Duration(15).sleep();

  // Call Service to enable position control for each joint
  std_srvs::Trigger srv;
  switchToPositionControllerServiceClient_.call(srv);

  ros::Duration(1).sleep();

  std::vector<std::string> jointNames =
      m_jointModelGroupPtr->getVariableNames();
  std::vector<double> currentJointPositions_;

  // Get current joint positions ( send every joint to 0)
  getJointPositions(jointNames, currentJointPositions_);
  // Send zeros to enable joints
  for (std::size_t i = jointNames.size() - 1; i + 1 > 0; --i) {

    std_msgs::Float64 jointCmd_;
    std_msgs::Float64 jointCmd1_;
    jointCmd1_.data = 0;
    jointCmd_.data = -currentJointPositions_[i];
    // jointCmd_.data = 0;
    //  Doesn't work as it should, something weird is happening,
    //  Either I get an emergency error
    //  Either it doesn't rotate
    //  Sometimes it rotates
    //  Wierd behaviour
    ROS_INFO("[ControlArm] Sending joint %s to  - %f", jointNames[i].c_str(),
             currentJointPositions_[i]);

    if (i == 5) {
      for (int k = 0; k < 5; k++) {
        cmdJoint6Publisher.publish(jointCmd_);
        ros::Duration(0.02).sleep();
      }
    }
    if (i == 4) {
      for (int k = 0; k < 5; k++) {
        cmdJoint5Publisher.publish(jointCmd_);
        ros::Duration(0.02).sleep();
      }
    }
    if (i == 3) {

      for (int k = 0; k < 5; k++) {
        cmdJoint4Publisher.publish(jointCmd_);
        ros::Duration(0.02).sleep();
      }
    }
    if (i == 2) {

      for (int k = 0; k < 5; k++) {
        cmdJoint3Publisher.publish(jointCmd_);
        ros::Duration(0.02).sleep();
      }
    }
    if (i == 1) {
      for (int k = 0; k < 5; k++) {
        cmdJoint2Publisher.publish(jointCmd_);
        ros::Duration(0.02).sleep();
      }
    }
    if (i == 0) {
      for (int k = 0; k < 5; k++) {
        cmdJoint1Publisher.publish(jointCmd1_);
        ros::Duration(0.02).sleep();
      }
    }

    float sleep_t = abs(currentJointPositions_[i] / 0.1); // MAX_SPEED RAD/S
    ros::Duration(sleep_t).sleep();
  }

  return true;
}

bool ControlArm::sendZeros(std::string ControllerType) {
  // NOTE: This method is used to activate joint control for each arm joint.
  // Joints do not move before recieving 0 as command, after recieving 0, you
  // can send anything and it should be fine --> Check schunk documentation for
  // this

  if (ControllerType == "position") {

    std_msgs::Float64 msg;
    msg.data = 0;

    // cmdJoint1Publisher.publish(msg);
    cmdJoint2Publisher.publish(msg);
    cmdJoint3Publisher.publish(msg);
    cmdJoint4Publisher.publish(msg);
    cmdJoint5Publisher.publish(msg);
    cmdJoint6Publisher.publish(msg);
  }
  if (ControllerType == "group") {
    std_msgs::Float64MultiArray msg;

    std::vector<double> activationPositions = {
        m_jointPositions_[0], m_jointPositions_[1], m_jointPositions_[2],
        m_jointPositions_[3], m_jointPositions_[4], 0};

    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = activationPositions.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "i";

    msg.data.clear();
    msg.data.insert(msg.data.end(), activationPositions.begin(),
                    activationPositions.end());

    cmdJointGroupPositionPublisher.publish(msg);
  }

  return true;
}

void ControlArm::getCurrentArmState() {

  // method is more like refresh current kinematic state
  // (getCurrentKinematicState)
  m_currentRobotStatePtr = m_moveGroupPtr->getCurrentState();
}

void ControlArm::getCurrentEndEffectorState(
    const std::string endEffectorLinkName) {

  m_endEffectorState =
      m_currentRobotStatePtr->getGlobalLinkTransform(endEffectorLinkName);

  bool debug = false;
  if (debug) {

    ROS_INFO_STREAM("Translation: \n"
                    << m_endEffectorState.translation() << "\n");
    ROS_INFO_STREAM("Rotation: \n" << m_endEffectorState.rotation() << "\n");
  }
}

void ControlArm::getJointPositions(const std::vector<std::string> &jointNames,
                                   std::vector<double> &jointGroupPositions) {

  m_currentRobotStatePtr->copyJointGroupPositions(m_jointModelGroupPtr,
                                                  jointGroupPositions);

  bool debug = true;
  if (debug) {
    for (std::size_t i = 0; i < jointNames.size(); ++i) {
      ROS_INFO("Joint %s: %f", jointNames[i].c_str(), jointGroupPositions[i]);
    }
  }
}


bool ControlArm::getIK(const geometry_msgs::Pose wantedPose, const std::size_t attempts, double timeout) {


  bool found_ik = m_currentRobotStatePtr->setFromIK(m_jointModelGroupPtr, wantedPose);


  bool debug = false;
  if (debug) {
    ROS_INFO("Found IK solution!");
  }

  return found_ik;
}

Eigen::MatrixXd ControlArm::getJacobian(Eigen::Vector3d refPointPosition) {

  Eigen::MatrixXd jacobianMatrix;
  m_currentRobotStatePtr->getJacobian(
      m_jointModelGroupPtr,
      m_currentRobotStatePtr->getLinkModel(
          m_jointModelGroupPtr->getLinkModelNames().back()),
      refPointPosition, jacobianMatrix);

  return jacobianMatrix;
}

float ControlArm::round(float var) {

  float value = (int)(var * 1000 + .5);
  return (float)value / 1000;
}

void ControlArm::run() {

  ros::Rate r(25);

  while (ros::ok) {
    // Get current joint position for every joint in robot arm
    getCurrentArmState();

    // Get all joints
    m_jointModelGroupPtr = m_currentRobotStatePtr->getJointModelGroup("arm");

    // Or use EndEffector link
    Eigen::Affine3d currentPose_ = m_moveGroupPtr->getCurrentState()->getFrameTransform("wsg50_center");
    geometry_msgs::Pose currentROSPose_; tf::poseEigenToMsg(currentPose_, currentROSPose_);
    currentPosePublisher_.publish(currentROSPose_);

    // Sleep
    r.sleep();

  }
}
