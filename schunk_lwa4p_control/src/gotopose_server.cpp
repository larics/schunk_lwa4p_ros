#include <ros/ros.h>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include "schunk_lwa4p_control/GoToPoseAction.h"


class GoToPoseActionServer{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<schunk_lwa4p_control::GoToPoseAction> as_;
        std::string action_name_;

        // subscribers
        ros::Subscriber currentPoseSubscriber;

        // action
        schunk_lwa4p_control::GoToPoseFeedback feedback_;
        schunk_lwa4p_control::GoToPoseResult result_;

    public:

    GoToPoseActionServer(std::string name) :
        as_(nh_, name, boost::bind(&GoToPoseActionServer::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
        initializeSubscribers();
    }

    ~GoToPoseActionServer(void)
    {
    }

    void initializeSubscribers()
    {
        currentPoseSubscriber = nh_.subscribe<geometry_msgs::Pose>("/control_arm_node/tool/current_pose", 10, &GoToPoseActionServer::currentPoseCB, this);

    }

    void currentPoseCB(const geometry_msgs::Pose::ConstPtr &msg)
    {
        feedback_.current_pose.position = msg->position;
        feedback_.current_pose.orientation = msg->orientation;
        as_.publishFeedback(feedback_);
    }

    float checkDist(geometry_msgs::Pose::ConstPtr &pose1, geometry_msgs::Pose::ConstPtr &pose2)
    {
        float x_dist = pow((pose1->position.x - pose2->position.x), 2);
        float y_dist = pow((pose1->position.y - pose2->position.y), 2);
        float z_dist = pow((pose1->position.z - pose2->position.z), 2);

        float dist = sqrt(x_dist + y_dist + z_dist);

        return dist;
    }


    void executeCB(const schunk_lwa4p_control::GoToPoseGoalConstPtr &goal)
    {

        goal->goal_pose;
        ROS_INFO("Goal received: !");
        ros::Rate(1);
        bool success = false;


        ROS_INFO("Minimum deviation is: %f", goal->minimum_deviation);
        ROS_INFO("Feedback is: %f", feedback_.current_pose.position.x);

        float epsilon = goal->minimum_deviation;


        while (checkDist(goal->goal_pose, feedback_.current_pose) > epsilon) {
            if (as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // Set the action state to preempted
                as_.setPreempted();
                break;
            }
        }
        /*
        while (checkDist(goal->goal_pose, feedback_.current_pose) > epsilon)
        {
            if ( as_.isPreemptRequested() || !ros::ok())
            {
                ROS_INFO("%s: Preempted", action_name_.c_str());
                // Set the action state to preempted
                as_.setPreempted();
                success = false;
                break;
            }
        }
         */

        if (success)
        {
            result_.reached_pose = true;
            ROS_INFO("%s: Succeded", action_name_.c_str());

            // set action state to succeded
            as_.setSucceeded(result_);
        }

    }




};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gotopose");

    GoToPoseActionServer gotopose("go_to_pose");
    ros::spin();
    return 0;

}