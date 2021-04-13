#include <ros/ros.h>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <std_srvs/Trigger.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/convert.h>
#include "schunk_lwa4p_control/SetupDistancerAction.h"

class SetupDistancerActionServer{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<schunk_lwa4p_control::SetupDistancerAction> as_;
    std::string action_name_;

    // publishers
    ros::Publisher cmdPosePublisher;
    ros::Publisher cmdOrientationPublisher;

    // subscribers
    ros::Subscriber currentPoseSubscriber;

    // service clients
    // ros::ServiceClient toolCmdServiceClient;
    ros::ServiceClient addCollisionsServiceClient;
    ros::ServiceClient disableToolCollisionsServiceClient;
    ros::ServiceClient setUpSeparatorServiceClient;

    // action
    schunk_lwa4p_control::SetupDistancerFeedback feedback_;
    schunk_lwa4p_control::SetupDistancerResult result_;
    schunk_lwa4p_control::SetupDistancerGoal goal_;

    // msgs
    geometry_msgs::Pose currentPose;
    geometry_msgs::Pose cmdPose;
    geometry_msgs::Point cmdOrientation;

    // wanted tool rotations
    tf2Scalar roll; tf2Scalar pitch; tf2Scalar yaw;


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
        ROS_INFO("[SetupDistancerServer] Initialized...");
        // Add some collisions (table for planning and working with real robot -> maybe not ideal here?)
        std_srvs::Trigger coll_srv;
        addCollisionsServiceClient.call(coll_srv);


    }

    ~SetupDistancerActionServer(void)
    {

    }


    void initializeSubscribers()
    {
        currentPoseSubscriber = nh_.subscribe<geometry_msgs::Pose>("/control_arm_node/tool/current_pose", 10, &SetupDistancerActionServer::currentPoseCB, this);

    }

    void initializePublishers()
    {
        cmdPosePublisher = nh_.advertise<geometry_msgs::Pose>("/control_arm_node/arm/command/pose", 1);
        cmdOrientationPublisher = nh_.advertise<geometry_msgs::Point>("/control_arm_node/tool/command/orientation", 1);
    }

    void initializeServices()
    {

        addCollisionsServiceClient = nh_.serviceClient<std_srvs::Trigger>("/control_arm_node/scene/add_collisions");
        disableToolCollisionsServiceClient = nh_.serviceClient<std_srvs::Trigger>("/control_arm_node/tool/disable_collision");
        // toolCmdServiceClient = nh_.serviceClient<separator_end_effector::separator_service>("/tool_service");

    }

    void currentPoseCB(const geometry_msgs::Pose::ConstPtr &msg)
    {
        // https://answers.ros.org/question/212857/what-is-constptr/
        currentPose.position = msg->position;
        currentPose.orientation = msg->orientation;

        tf2::Quaternion quaternion(currentPose.orientation.x, currentPose.orientation.y, currentPose.orientation.z, currentPose.orientation.w);

        // Get quaternion from ROS msg into tf2 quaternion
        tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

        //ROS_INFO_STREAM("[SetupDistancer] Roll is " << roll);
        //ROS_INFO_STREAM("[SetupDistancer] Pitch is: " << pitch);
        //ROS_INFO_STREAM("[SetupDistancer] Yaw is: " << yaw);


        // Get tool orientation from current pose (and compare to MoveIt method used in control_arm.cpp

    }

    void executeCB(const schunk_lwa4p_control::SetupDistancerGoalConstPtr &goal)
    {

        ros::Rate r(2);

        ROS_INFO("[SetupDistancerServer] Received new goal!");

        bool sentCmd = false;
        bool elapsed = false;
        bool preempted = false;
        // 1. Disable tool collisions
        bool disabled_tool_collisions = false;
        // 2. Rotate tool
        bool orientation_cmd_sent = false;
        bool tool_rotation_completed = false;
        // 3. Close motors
        bool called_close_motors_srv = false;
        bool separator_on_powerlines = false;

        int tRecvGoal = ros::Time::now().toSec();
        int timeout = goal->timeout_sec;
        float epsilon = goal->epsilon;
        r.sleep();

        while (!elapsed && !preempted) {
            r.sleep();
            elapsed = (ros::Time::now().toSec() - tRecvGoal) > timeout;

            // Remove collisions for tool
            std_srvs::Trigger srv;
            if (!disabled_tool_collisions) {
                ROS_INFO("[SetupDistancerServer] Removing tool collisions.");
                disableToolCollisionsServiceClient.call(srv);
                disabled_tool_collisions = true;
            }

            if (!orientation_cmd_sent) {
                    ROS_INFO("[SetupDistancerServer] Starting tool rotation..");
                    cmdOrientation = goal->goal_orientation;
                    cmdOrientationPublisher.publish(cmdOrientation); // This gets published but somehow trajectory Start fails
                    orientation_cmd_sent = true;
            }

            // This condition may fail because of type comparison, check how to transform tf2scalar to float
            if (cmdOrientation.z -  yaw > epsilon){
                ROS_INFO("[SetupDistancerServer] Rotating tool...");
                tool_rotation_completed = false;
            } else{
                ROS_INFO("[SetupDistancerServer] Rotation complete.");
                tool_rotation_completed = true;
            }

            if (tool_rotation_completed && !called_close_motors_srv) {
                std_srvs::Trigger srv;
                setUpSeparatorServiceClient.call(srv);
                called_close_motors_srv = true;
                // Wait for response maybe (How to wait for response)
                // For starters close separator
                // Could eventually monitor force with which motors close it
            }

            // TODO: Add method check for separator on powerlines

            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                preempted = true;
            }
        }

        if (elapsed && !preempted)
        {
            ROS_INFO("[SetupDistancerServer] Timeout reached: ABORTED");
            as_.setAborted(result_);
        }
        if (!preempted && !elapsed && tool_rotation_completed && separator_on_powerlines)
        {
            result_.distancer_on_powerlines = true;
            as_.setSucceeded(result_);

            ROS_INFO("[SetupDistancerServer] Reached wanted pose: SUCCEEDDED!");
        }


        // TODO:
        // 1. Build new action and start new action server --> DONE
        // 2. Command tool service based on received goal --> DONE
        // 3. Command tool orientation based on received goal (Doesen't move when commanded from server?!) --> Fixed WHILE
        // 4. Add Trigger service call to enable closing motors on trigger
        // 5. Create sequence of service calls (remove_collision, send_orientation, close motors)
        // 6. Check orientation in which format it's written (radians/degrees)
        // Think of arm servoing as a service moveit::servo as service that can be called during executing of this server


    }




};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "setup_distancer");

    SetupDistancerActionServer setup_distancer("setup_distancer");
    ros::spin();
    return 0;

}