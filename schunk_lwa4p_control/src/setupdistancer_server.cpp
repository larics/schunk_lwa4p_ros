#include <ros/ros.h>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>
#include "schunk_lwa4p_control/SetupDistancerAction.h"

class SetupDistancerActionServer{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<schunk_lwa4p_control::SetupDistancerAction> as_;
    std::string action_name_;

    // publishers
    ros::Publisher cmdPosePublisher;

    // subscribers
    ros::Subscriber currentPoseSubscriber;

    // service clients
    // ros::ServiceClient toolCmdServiceClient;
    ros::ServiceClient addCollisionsServiceClient;
    ros::ServiceClient disableToolCollisionsServiceClient;

    // action
    schunk_lwa4p_control::SetupDistancerFeedback feedback_;
    schunk_lwa4p_control::SetupDistancerResult result_;
    schunk_lwa4p_control::SetupDistancerGoal goal_;

    // msgs
    geometry_msgs::Pose currentPose;
    geometry_msgs::Pose cmdPose;


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

        // Get tool orientation from current pose (and compare to MoveIt method used in control_arm.cpp

    }



    void executeCB(const schunk_lwa4p_control::SetupDistancerGoalConstPtr &goal)
    {

        ros::Rate r(2);

        ROS_INFO("[GoToPose] Received new goal!");

        bool sentCmd = false;
        bool elapsed = false;
        bool reached = false;
        bool preempted = false;
        int tRecvGoal = ros::Time::now().toSec();
        r.sleep();

        // TODO:
        // 1. Build new action and start new action server
        // 2. Command tool service based on received goal
        // 3. Command tool orientation based on received goal
        // 4. Create sequence of service calls (remove_collision, send_orientation, close motors)


        /*

        // Feedback publishing
        feedback_.current_pose.position = currentPose.position;
        feedback_.current_pose.orientation = currentPose.orientation;
        as_.publishFeedback(feedback_);

        // Goal variables
        cmdPose = static_cast<geometry_msgs::Pose>(goal->goal_pose);
        float epsilon = goal->minimum_deviation;
        int timeout = goal->timeout_sec;

        //

        while (checkDist(cmdPose, currentPose) > epsilon && !elapsed){
            // Check timeout condition
            r.sleep();
            elapsed = ( ros::Time::now().toSec() - tRecvGoal) > timeout;
            // Send to pose
            if(!sentCmd){
                cmdPosePublisher.publish(cmdPose);
                sentCmd = true;
                ROS_INFO("[GoToPose] Sending command!");
            }

            // Check preemption
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // set the action state to preempted
                as_.setPreempted();
                reached = false;
                preempted = true;
            }
        }

        if(checkDist(cmdPose, currentPose) < epsilon){
            reached = true;
        }

        result_.reached_pose = reached;
        if (elapsed || preempted)
        {
            ROS_INFO("[GoToPose] Timeout reached: ABORTED");
            as_.setAborted(result_);
        }
        else
        {
            result_.reached_pose = true;
            as_.setSucceeded(result_);
            ROS_INFO("[GoToPose] Reached wanted pose: SUCCEEDED!");
        }

        */

    }




};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "setup_distancer");

    SetupDistancerActionServer setup_distancer("setup_distancer");
    ros::spin();
    return 0;

}