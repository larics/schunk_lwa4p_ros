#ifndef CONTROL_ARM_H
#define CONTROL_ARM_H

#include <chrono>
#include <cmath>
#include <iostream>
#include <pthread.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

// ROS
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// MoveIt
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionIKRequest.h>
#include <moveit_msgs/GetPositionIKResponse.h>
#include <moveit_msgs/PositionIKRequest.h>

// Conversions
#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

// Utils
#include <tf2/LinearMath/Quaternion.h>

// Custom services
#include "schunk_lwa4p_control/getIk.h"
#include "schunk_lwa4p_control/getIkRequest.h"
#include "schunk_lwa4p_control/getIkResponse.h"


class ControlArm {

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

  // Pointer to move group
  // (https://answers.ros.org/question/344598/cant-create-movegroupinterface-object-in-my-own-class/)
  moveit::planning_interface::MoveGroupInterface *m_moveGroupPtr;
  const robot_state::JointModelGroup *m_jointModelGroupPtr;
  planning_scene::PlanningScene *m_planningScenePtr;
  moveit::core::RobotStatePtr m_currentRobotStatePtr;
  robot_model_loader::RobotModelLoader m_robotModelLoader;
  Eigen::Affine3d m_endEffectorState;

  // Run method
  void run();

  // It's not possible to access private members of this class from another clas
  // Link to issue that explains it:
  // https://stackoverflow.com/questions/18944451/how-to-make-a-derived-class-access-the-private-member-data
private:
  // Reads and verifies ROS parameters
  bool readParameters();

  // Initialization method
  void init();

  // ROS node handle
  ros::NodeHandle nodeHandle_;
  ros::NodeHandle nodeHandleWithoutNs_;

  // transformListener
  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  // ROS Publishers
  ros::Publisher displayTrajectoryPublisher_;
  ros::Publisher currentPosePublisher_;
  ros::Publisher cmdJoint1Publisher;
  ros::Publisher cmdJoint2Publisher;
  ros::Publisher cmdJoint3Publisher;
  ros::Publisher cmdJoint4Publisher;
  ros::Publisher cmdJoint5Publisher;
  ros::Publisher cmdJoint6Publisher;
  ros::Publisher cmdJointGroupPositionPublisher;
  ros::Publisher cmdJointGroupVelocityPublisher;
  ros::Publisher powerline0PosePublisher;
  ros::Publisher powerline1PosePublisher;

  // ROS Subscribers
  ros::Subscriber armCmdPoseSubscriber_;
  ros::Subscriber armCmdDeltaPoseSubscriber_;
  ros::Subscriber armCmdToolOrientationSubscriber_;

  // ROS Services
  ros::ServiceServer disableCollisionService_;
  ros::ServiceServer addCollisionObjectService_;
  ros::ServiceServer startPositionControllersService_;
  ros::ServiceServer startJointTrajectoryControllerService_;
  ros::ServiceServer startJointGroupPositionControllerService_;
  ros::ServiceServer startJointGroupVelocityControllerService_;
  ros::ServiceServer sendArmToHomingPoseService_;
  ros::ServiceServer getIkService_;


  // ROS Service clients
  ros::ServiceClient applyPlanningSceneServiceClient_;
  ros::ServiceClient realRobotDriverInitServiceClient_;
  ros::ServiceClient addCollisionObjectServiceClient_;
  ros::ServiceClient switchControllerServiceClient_;
  ros::ServiceClient listControllersServiceClient_;
  ros::ServiceClient switchToPositionControllerServiceClient_;
  ros::ServiceClient switchToTrajectoryControllerServiceClient_;

  // ROS Subscriber Callback
  void cmdPoseCallback(const geometry_msgs::Pose::ConstPtr &msg);
  void cmdDeltaPoseCallback(const geometry_msgs::Pose::ConstPtr &msg);
  void cmdToolOrientationCallback(const geometry_msgs::Point::ConstPtr &msg);

  // ROS Services callbacks
  bool disableCollisionServiceCallback(std_srvs::TriggerRequest &req,
                                       std_srvs::TriggerResponse &res);


  bool addCollisionObjectServiceCallback(std_srvs::TriggerRequest &req,
                                         std_srvs::TriggerResponse &res);
  bool startPositionControllers(std_srvs::TriggerRequest &req,
                                std_srvs::TriggerResponse &res);
  bool startJointTrajectoryController(std_srvs::TriggerRequest &req,
                                      std_srvs::TriggerResponse &res);
  bool startJointGroupPositionController(std_srvs::TriggerRequest &req,
                                         std_srvs::TriggerResponse &res);
  bool startJointGroupVelocityController(std_srvs::TriggerRequest &req,
                                         std_srvs::TriggerResponse &res);
  bool getIkServiceCallback(schunk_lwa4p_control::getIkRequest &req,
                            schunk_lwa4p_control::getIkResponse &res);
  bool sendArmToHomingPose(std_srvs::TriggerRequest &req,
                           std_srvs::TriggerResponse &res);

  // DisplayTrajectory
  moveit_msgs::DisplayTrajectory displayTrajectory_;

  // Private variables
  int sleepMs_;
  bool realRobot_;
  bool enableVisualization_;
  bool moveGroupInitialized_;
  bool planningSceneInitialized_;
  bool blockingMovement = true;
  geometry_msgs::Pose m_cmdPose;
  geometry_msgs::Pose m_lastCmdPose;
  geometry_msgs::Pose m_cmdDeltaPose;

  // Vectors and arrays
  std::vector<double> m_jointPositions_;

  bool sendZeros(std::string ControllerType);
  bool sendToCmdPose();
  void sendToCmdPoses(std::vector<geometry_msgs::Pose> poses);
  bool sendToDeltaCmdPose();
  void addCollisionObject(moveit_msgs::PlanningScene &planningScene);
  void getCurrentArmState();
  void getCurrentEndEffectorState(const std::string linkName);
  void getJointPositions(const std::vector<std::string> &jointNames,
                         std::vector<double> &jointGroupPositions);
  void getRunningControllers(std::vector<std::string> &runningControllerNames);
  bool getIK(const geometry_msgs::Pose wantedPose, const std::size_t attempts, double timeout);
  Eigen::MatrixXd getJacobian(Eigen::Vector3d
                  refPointPosition); // Can be created as void and arg passed to
                                     // be changed during execution
  Eigen::MatrixXd getInertiaMatrix(Eigen::Vector3d refPointPosition);

  // TODO: Move this to utils.cpp
  float round(float var);

};

#endif // CONTROL_ARM_H
