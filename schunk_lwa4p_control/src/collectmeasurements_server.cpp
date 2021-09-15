//
// Created by developer on 9/14/21.
//
#include <ros/ros.h>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

// action specific stuff
#include "schunk_lwa4p_control/CollectMeasurementsAction.h"

class CollectMeasurementsServer{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<schunk_lwa4p_control::CollectMeasurementsAction> as_;
        std::string action_name_;

        // Publisher
        ros::Publisher mangeticEstimatePosePub;

        // Action
        schunk_lwa4p_control::CollectMeasurementsFeedback feedback_;
        schunk_lwa4p_control::CollectMeasurementsResult result_;
        schunk_lwa4p_control::CollectMeasurementsGoal goal_;

        // TransformListener
        tf::TransformListener listener;

        // Msgs
        geometry_msgs::Pose currMeasuredPose;

    public:

    CollectMeasurementsServer(std::string name) :
        as_(nh_, name, boost::bind(&CollectMeasurementsServer::executeCB, this, _1), false),
        action_name_(name)

        {

            initializePublishers();
            as_.start();

        }

    ~CollectMeasurementsServer(void)
    {
    }


    void initializePublishers()
    {
        mangeticEstimatePosePub = nh_.advertise<geometry_msgs::Pose>("/magnetic_estimate", 1);
    }

    void currentMagPoseEstCB(const geometry_msgs::Pose::ConstPtr &msg)
    {
        // https://answers.ros.org/question/212857/what-is-constptr/
        currMeasuredPose.position = msg->position;
        currMeasuredPose.orientation = msg->orientation;

    }

    // TODO: Fix transition between state 2 and 3
    void executeCB(const schunk_lwa4p_control::CollectMeasurementsGoalConstPtr &goal)
    {

        ros::Rate r(50);

        ROS_INFO("[CollectMeasurementServer] Received new goal!");

        bool elapsed = false;
        bool preempted = false;
        bool succeeded = false;
        bool measured = false;
        int tRecvGoal = ros::Time::now().toSec();

        r.sleep();

        // Feedback publishing
        feedback_.current_estimate.position = currMeasuredPose.position;
        feedback_.current_estimate.orientation = currMeasuredPose.orientation;
        as_.publishFeedback(feedback_);

        // Goal variables
        int timeout = goal->timeout_sec;

        while ((!measured && !elapsed && !preempted)){

            tf::StampedTransform transform1; tf::StampedTransform transform2;
            listener.lookupTransform("base_link", "power_line0", ros::Time(0), transform1);
            listener.lookupTransform("base_link", "power_line1", ros::Time(0), transform2);

            geometry_msgs::PoseStamped magnetic_pose;

            magnetic_pose.pose.position.x = (transform1.getOrigin().x() + transform2.getOrigin().x()) / 2;
            magnetic_pose.pose.position.y = (transform1.getOrigin().y() + transform2.getOrigin().y()) / 2;
            magnetic_pose.pose.position.z = (transform1.getOrigin().z() + transform2.getOrigin().z()) / 2;
            magnetic_pose.pose.orientation.x = transform1.getRotation().x();
            magnetic_pose.pose.orientation.y = transform1.getRotation().y();
            magnetic_pose.pose.orientation.z = transform1.getRotation().z();
            magnetic_pose.pose.orientation.w = transform1.getRotation().w();

            mangeticEstimatePosePub.publish(magnetic_pose);

            // Check timeout condition
            r.sleep();
            elapsed = ( ros::Time::now().toSec() - tRecvGoal) > timeout;

            // TODO: Subscribe to collect available measurements

            // Check preemption
            if ( as_.isPreemptRequested() || !ros::ok() )
            {
                ROS_INFO("[CollectMeasurementsServer] Preempted");
                // set the action state to preempted
                as_.setPreempted();
                preempted = true;
                elapsed = false;
            }
        }

        if (elapsed && !preempted)
        {
            ROS_INFO("[CollectMeasurementsServer] Timeout reached: ABORTED");
            as_.setAborted(result_);
        }
        if (!succeeded && !preempted)
        {
            succeeded = true;
            //result_.reached_pose = true;
            // Added 10 seconds sleep to wait for recording magnetic measurements during execution
            // sleep(10);
            as_.setSucceeded(result_);

            ROS_INFO("[CollectMeasurementsServer] Reached wanted pose: SUCCEEDDED!");
        }
    }




};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collectmeasurements");

    CollectMeasurementsServer collectmeasurements("collect_measurements");
    ros::spin();
    return 0;

}
