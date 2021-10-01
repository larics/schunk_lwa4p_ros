#include <ros/ros.h>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>

// Specific includes
#include "schunk_lwa4p_control/SetupDistancerAction.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"
// TODO: Include separator service
#include "separator_end_effector/separator_service.h"
#include "separator_end_effector/separator_serviceRequest.h"
#include "separator_end_effector/separator_serviceResponse.h"


class SetupDistancerActionServer{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<schunk_lwa4p_control::SetupDistancerAction> as_;
    std::string action_name_;

    // publishers
    ros::Publisher cmdPosePublisher;
    ros::Publisher cmdOrientationPublisher;
    ros::Publisher cmdLwa4pJoint6Publisher;
    ros::Publisher cmdLeftDistancerPublisher;
    ros::Publisher cmdRightDistancerPublisher;

    // subscribers
    ros::Subscriber currentPoseSubscriber;
    ros::Subscriber dynamixelStateSubscriber;
    ros::Subscriber jointStateSubscriber;

    // service clients
    ros::ServiceClient addCollisionsServiceClient;
    ros::ServiceClient disableToolCollisionsServiceClient;
    ros::ServiceClient startJointPositionControllersClient;
    ros::ServiceClient startJointTrajectoryControllerClient;
    ros::ServiceClient toolCmdServiceClient;
    ros::ServiceClient recoverDriverClient;

    // action
    schunk_lwa4p_control::SetupDistancerFeedback feedback_;
    schunk_lwa4p_control::SetupDistancerResult result_;
    schunk_lwa4p_control::SetupDistancerGoal goal_;

    // msgs
    geometry_msgs::Pose currentPose;
    geometry_msgs::Point cmdOrientation;
    sensor_msgs::JointState jointState;

    // services
    separator_end_effector::separator_serviceRequest separatorServiceReq;
    separator_end_effector::separator_serviceResponse separatorServiceRes;

    // wanted tool rotations
    tf2Scalar roll; tf2Scalar pitch; tf2Scalar yaw;

    // real robot
    bool realRobot;
    int rightMotor; float rightMotorPosition;
    int leftMotor; float leftMotorPosition;
    float yaw_;


public:

    SetupDistancerActionServer(std::string name) :
            as_(nh_, name, boost::bind(&SetupDistancerActionServer::executeCB, this, _1), false),
            action_name_(name)
    {
        // Call driver to start motors!

        initializeSubscribers();
        initializePublishers();
        initializeServices();
        as_.start();
        nh_.getParam("real_robot", realRobot);
        ROS_INFO("[SetupDistancerServer] Initialized...");
        // Add some collisions (table for planning and working with real robot -> maybe not ideal here?)
        std_srvs::Trigger coll_srv;
        addCollisionsServiceClient.call(coll_srv);


    }

    ~SetupDistancerActionServer(void)
    {

    }


    void initializeSubscribers(){
        currentPoseSubscriber = nh_.subscribe<geometry_msgs::Pose>("/control_arm_node/tool/current_pose", 10, &SetupDistancerActionServer::currentPoseCB, this);
        dynamixelStateSubscriber = nh_.subscribe<dynamixel_workbench_msgs::DynamixelStateList>("/dynamixel_workbench/dynamixel_state", 10, &SetupDistancerActionServer::dynamixelStateCB, this);
        jointStateSubscriber = nh_.subscribe<sensor_msgs::JointState>("/lwa4p/joint_states", 10, &SetupDistancerActionServer::jointStateCB, this);

    }

    void initializePublishers(){
        cmdOrientationPublisher = nh_.advertise<geometry_msgs::Point>("/control_arm_node/tool/command/orientation", 1);
        cmdLwa4pJoint6Publisher =  nh_.advertise<std_msgs::Float64>("/lwa4p/joint_6_position_controller/command", 1);
        cmdLeftDistancerPublisher = nh_.advertise<std_msgs::Float64>("/lwa4p/distancer_left_position_controller/command", 1);
        cmdRightDistancerPublisher = nh_.advertise<std_msgs::Float64>("/lwa4p/distancer_right_position_controller/command", 1);
    }

    void initializeServices(){

        addCollisionsServiceClient = nh_.serviceClient<std_srvs::Trigger>("/control_arm_node/scene/add_collisions");
        disableToolCollisionsServiceClient = nh_.serviceClient<std_srvs::Trigger>("/control_arm_node/tool/disable_collision");
        startJointPositionControllersClient = nh_.serviceClient<std_srvs::Trigger>("/control_arm_node/controllers/start_position_controllers");
        startJointTrajectoryControllerClient = nh_.serviceClient<std_srvs::Trigger>("/control_arm_node/controllers/start_joint_trajectory_controller");
        recoverDriverClient = nh_.serviceClient<std_srvs::Trigger>("/lwa4p/driver/recover");
        toolCmdServiceClient = nh_.serviceClient<separator_end_effector::separator_service>("/tool_service");

    }

    void jointStateCB(const sensor_msgs::JointStateConstPtr &msg){

        yaw_ = msg->position[5];
    }

    void dynamixelStateCB(const dynamixel_workbench_msgs::DynamixelStateListConstPtr &msg) {
        for (size_t i = 0; i != msg->dynamixel_state.size(); i++) {
            if (msg->dynamixel_state[i].name == "right") {
                rightMotor = msg->dynamixel_state[i].present_current;
                rightMotorPosition = msg->dynamixel_state[i].present_position;
            }

            if (msg->dynamixel_state[i].name == "left") {
                leftMotor = msg->dynamixel_state[i].present_current;
                leftMotorPosition = msg->dynamixel_state[i].present_position;
            }


        }
    }

    void currentPoseCB(const geometry_msgs::Pose::ConstPtr &msg) {
            // https://answers.ros.org/question/212857/what-is-constptr/
            currentPose.position = msg->position;
            currentPose.orientation = msg->orientation;

            tf2::Quaternion quaternion(currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z,
                                       currentPose.orientation.w);

            // Get quaternion from ROS msg into tf2 quaternion
            tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

            //ROS_INFO_STREAM("[SetupDistancer] Roll is " << roll);
            //ROS_INFO_STREAM("[SetupDistancer] Pitch is: " << pitch);
            //ROS_INFO_STREAM("[SetupDistancer] Yaw is: " << yaw);


            // Get tool orientation from current pose (and compare to MoveIt method used in control_arm.cpp

    }

    void executeCB(const schunk_lwa4p_control::SetupDistancerGoalConstPtr &goal) {

            ros::Rate r(2);

            ROS_INFO("[SetupDistancerServer] Received new goal!");

            bool sentCmd = false;
            bool elapsed = false;
            bool preempted = false;
            int i = 0;
            // 1. Disable tool collisions
            bool disabled_tool_collisions = false;
            // 2. Rotate tool
            bool orientation_cmd_sent = false;
            bool tool_rotation_completed = false;
            // 3. Close motors
            bool called_close_motors_srv = false;
            bool separator_on_powerlines = false;
            // 4. Check if motors have been closed
            bool closed_right = false;
            bool closed_left = false;
            bool closed_both = false;
            // 5. Pull arm back down in some linear motion --> servoing
            std_msgs::Float64 jointCmd;
            int tRecvGoal = ros::Time::now().toSec();
            int timeout = goal->timeout_sec;
            float epsilon = goal->epsilon;
            r.sleep();

            while (!elapsed && !preempted) {
                r.sleep();
                elapsed = (ros::Time::now().toSec() - tRecvGoal) > timeout;

                // Remove collisions for tool
                std_srvs::Trigger srv;

                float wanted_rotation = goal->goal_orientation.z;
                float cmd_ = -yaw_;
                if (!disabled_tool_collisions) {
                    ROS_INFO("[SetupDistancerServer] Removing tool collisions.");
                    disableToolCollisionsServiceClient.call(srv);
                    disabled_tool_collisions = true;
                }

                if (!orientation_cmd_sent) {
                    ROS_INFO("[SetupDistancerServer] Starting tool rotation..");

                    separatorServiceReq.req = "open_both";
                    toolCmdServiceClient.call(separatorServiceReq, separatorServiceRes);

                    // call services to load position controllers
                    std_srvs::Trigger srv;
                    startJointPositionControllersClient.call(srv);
                    ros::Duration(3.0).sleep();
                    ROS_INFO_STREAM("[SetupDistancerServer] Response: " << srv.response);

                    // Wait to reload controllers
                    jointCmd.data = 0.0;
                    cmdLwa4pJoint6Publisher.publish(jointCmd);
                    ros::Duration(1.0).sleep();
                    jointCmd.data = goal->goal_orientation.z; // 90° currently, but should be relative to current orientation
                    orientation_cmd_sent = true;
                }

                // Yaw_ is current measurement for last joint
                // TODO: Check different condition to rotate Tool for 90 degrees
                if ((abs(yaw_ - wanted_rotation) > epsilon)) {

                    // Added this condition to prevent tool rotation in certain configuration which could damage tool
                    if (roll > 0.1 || pitch > 0.1) {
                        as_.setAborted(result_);
                    }
                    //TODO: Check this, doesn't work as it should
                    ROS_INFO("[SetupDistancerServer] Rotating tool...");
                    ROS_INFO_STREAM("[SetupDistancerServer] Yaw calculated: " << yaw);
                    ROS_INFO_STREAM("[SetupDistancerServer] Yaw measured:" << yaw_);
                    ROS_INFO_STREAM("[SetupDistancerServer] Diff value: " << abs(yaw_ - wanted_rotation));
                    ROS_DEBUG_STREAM("[SetupDistancerServer] Publishing : " << jointCmd.data);
                    ROS_DEBUG_STREAM("[SetupDistancerServer] Roll: " << roll << "Pitch: " << pitch);

                    if (i < 5) {
                        jointCmd.data = 0.0;
                        cmdLwa4pJoint6Publisher.publish(jointCmd);  // This is in radians
                        i += 1;

                        // Commented this out to prevent unneccessary opening of a separator tool
                        separatorServiceReq.req = "open_both";
                        toolCmdServiceClient.call(separatorServiceReq, separatorServiceRes);
                        // Could possibly add opening of a tool

                    } else {
                        separatorServiceReq.req = "stop";
                        toolCmdServiceClient.call(separatorServiceReq, separatorServiceRes);
                        jointCmd.data = cmd_;
                        cmdLwa4pJoint6Publisher.publish(jointCmd);
                        ROS_INFO_STREAM("Passing...");
                    }


                } else {
                    ROS_INFO("[SetupDistancerServer] Rotation complete.");
                    ROS_INFO_STREAM("[SetupDistancerServer] called_close_motors_srv" << called_close_motors_srv);
                    tool_rotation_completed = true;
                }

                if (tool_rotation_completed && (!closed_right || !closed_left)) {
                    ROS_INFO("[SetupDistancerServer] Setting distancer on powerlines...");

                    // Wait for response maybe (How to wait for response)
                    // For starters close separator
                    // Could eventually monitor force with which motors close it
                    if (!realRobot) {
                        ROS_INFO("[SetupDistancerServer] Closing distancer...");
                        //TODO: Get current joint positions for that motor
                        std_msgs::Float64 rightDistancerCmd;
                        rightDistancerCmd.data = 0.5;
                        std_msgs::Float64 leftDistancerCmd;
                        leftDistancerCmd.data = -0.5;
                        cmdRightDistancerPublisher.publish(rightDistancerCmd);
                        cmdLeftDistancerPublisher.publish(leftDistancerCmd);
                    } else {

                        ROS_INFO_STREAM("[SetupDistancerServer] Entered else!");
                        separatorServiceReq.req = "close_both";
                        toolCmdServiceClient.call(separatorServiceReq, separatorServiceRes);

                        ROS_INFO_STREAM("rightMotor: " << rightMotor);
                        ROS_INFO_STREAM("leftMotor: " << leftMotor);
                        if (rightMotor > 1150) {
                            separatorServiceReq.req = "stop1";
                            toolCmdServiceClient.call(separatorServiceReq, separatorServiceRes);
                            closed_right = true;
                        }
                        if (leftMotor > 1150) {
                            separatorServiceReq.req = "stop2";
                            toolCmdServiceClient.call(separatorServiceReq, separatorServiceRes);
                            closed_left = true;
                        }

                        closed_both = (closed_right && closed_left);

                    }
                }

                // TODO: Add method check for separator on powerlines

                if (as_.isPreemptRequested() || !ros::ok()) {
                    ROS_INFO("%s: Preempted", action_name_.c_str());
                    // set the action state to preempted
                    as_.setPreempted();
                    preempted = true;
                }

                if (elapsed && !preempted) {
                    ROS_INFO("[SetupDistancerServer] Timeout reached: ABORTED");
                    as_.setAborted(result_);
                }

                if (!preempted && !elapsed && tool_rotation_completed &&  closed_both) {

                    std_srvs::Trigger trajectorySrv;
                    // Activate JointGroupPositionController
                    //.call(trajectorySrv);
                    recoverDriverClient.call(trajectorySrv);
                    ros::Duration(1.0).sleep();
                    result_.distancer_on_powerlines = true;
                    as_.setSucceeded(result_);
                    ROS_INFO("[SetupDistancerServer] Reached wanted pose: SUCCEEDDED!");
                }
            }


        }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "setup_distancer");

    SetupDistancerActionServer setup_distancer("setup_distancer");
    ros::spin();
    return 0;

};