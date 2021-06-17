#include <ros/ros.h>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/Pose.h>
#include <std_srvs/Trigger.h>
#include "schunk_lwa4p_control/GoToPoseAction.h"


class GoToPoseActionServer{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<schunk_lwa4p_control::GoToPoseAction> as_;
        std::string action_name_;

        // publishers
        ros::Publisher cmdPosePublisher;

        // subscribers
        ros::Subscriber currentPoseSubscriber;

        // service clients
        ros::ServiceClient realRobotDriverInitServiceClient_;

        ros::ServiceClient addCollisionObjectServiceClient_;

        // action
        schunk_lwa4p_control::GoToPoseFeedback feedback_;
        schunk_lwa4p_control::GoToPoseResult result_;
        schunk_lwa4p_control::GoToPoseGoal goal_;

        // msgs
        geometry_msgs::Pose currentPose;
        geometry_msgs::Pose cmdPose;

        // flags
        bool realRobot;

    public:

    GoToPoseActionServer(std::string name) :
        as_(nh_, name, boost::bind(&GoToPoseActionServer::executeCB, this, _1), false),
        action_name_(name)
    {
        // Call driver to start motors!
        nh_.getParam("real_robot", realRobot);
        if (realRobot)
        {
            startRealRobot();
        }else {
            ROS_INFO("[GoToPoseServer] Starting simulation...");
        }
        initializeSubscribers();
        initializePublishers();
        as_.start();

    }

    ~GoToPoseActionServer(void)
    {
    }

    void startRealRobot()
    {
        ROS_INFO("[GoToPoseServer] Starting real robot...");
        realRobotDriverInitServiceClient_ = nh_.serviceClient<std_srvs::Trigger>("/lwa4p/driver/init");
        realRobotDriverInitServiceClient_.waitForExistence();
        std_srvs::Trigger srv;
        realRobotDriverInitServiceClient_.call(srv);

        addCollisionObjectServiceClient_ = nh_.serviceClient<std_srvs::Trigger>("/control_arm_node/scene/add_collisions");
        addCollisionObjectServiceClient_.waitForExistence();
        addCollisionObjectServiceClient_.call(srv);
    }

    void initializeSubscribers()
    {
        currentPoseSubscriber = nh_.subscribe<geometry_msgs::Pose>("/control_arm_node/tool/current_pose", 10, &GoToPoseActionServer::currentPoseCB, this);

    }

    void initializePublishers()
    {
        cmdPosePublisher = nh_.advertise<geometry_msgs::Pose>("/control_arm_node/arm/command/pose", 1);
    }

    void currentPoseCB(const geometry_msgs::Pose::ConstPtr &msg)
    {
        // https://answers.ros.org/question/212857/what-is-constptr/
        currentPose.position = msg->position;
        currentPose.orientation = msg->orientation;

    }

    //TODO: Add checkOrientationDist

    float checkOrientationDist(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2){

        // quaternion distance: https://math.stackexchange.com/questions/90081/quaternion-distance
        float a1 = pose1.orientation.w; float a2 = pose2.orientation.w;
        float b1 = pose1.orientation.x; float b2 = pose2.orientation.x;
        float c1 = pose1.orientation.y; float c2 = pose2.orientation.y;
        float d1 = pose1.orientation.z; float d2 = pose2.orientation.z;

        float dist = 1 - (a1*a2 + b1*b2 + c1*c2 + d1*d2);

        //0 whenenver the quaternions represent the same orientation, 1 when they're 180 apart
        return dist;
    }

    float checkDist(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
    {
        float x_dist = pow((pose1.position.x - pose2.position.x), 2);
        float y_dist = pow((pose1.position.y - pose2.position.y), 2);
        float z_dist = pow((pose1.position.z - pose2.position.z), 2);

        // float x_ang_dist = pow((pose1.orientation.x - pose2.orientation.x), 1/2);
        // float y_ang_dist = pow((pose1.orientation.y - pose2.orientation.y), 1/2);
        // float z_ang_dist = pow((pose1.orientation.z - pose2.orientation.z), 1/2);
        // float w_ang_dist = pow((pose1.orientation.w - pose2.orientation.w), 1/2);

        float dist = sqrt(x_dist + y_dist + z_dist);

        return dist;
    }

    // TODO: Fix transition between state 2 and 3
    void executeCB(const schunk_lwa4p_control::GoToPoseGoalConstPtr &goal)
    {

        ros::Rate r(2);

        ROS_INFO("[GoToPoseServer] Received new goal!");

        bool sentCmd = false;
        bool elapsed = false;
        bool reached = false;
        bool preempted = false;
        bool succeeded = false;
        int tRecvGoal = ros::Time::now().toSec();

        r.sleep();

        // Feedback publishing
        feedback_.current_pose.position = currentPose.position;
        feedback_.current_pose.orientation = currentPose.orientation;
        as_.publishFeedback(feedback_);

        // Goal variables
        cmdPose = static_cast<geometry_msgs::Pose>(goal->goal_pose);
        float epsilon = goal->minimum_deviation;
        int timeout = goal->timeout_sec;

        while ((checkDist(cmdPose, currentPose)  + checkOrientationDist(cmdPose, currentPose))> epsilon && !elapsed && !preempted){
            // Check timeout condition
            r.sleep();
            elapsed = ( ros::Time::now().toSec() - tRecvGoal) > timeout;
            // Send to pose
            if(!sentCmd){
                cmdPosePublisher.publish(cmdPose);
                sentCmd = true;
                ROS_INFO("[GoToPoseServer] Sending command!");
            }

            // Check preemption
            if ( as_.isPreemptRequested() || !ros::ok() )
            {
                ROS_INFO("[GoToPoseServer] Preempted");
                // set the action state to preempted
                as_.setPreempted();
                reached = false;
                preempted = true;
                elapsed = false;
            }
        }

        if(checkDist(cmdPose, currentPose) < epsilon){
            reached = true;
        }

        result_.reached_pose = reached;
        if (elapsed && !preempted)
        {
            ROS_INFO("[GoToPoseServer] Timeout reached: ABORTED");
            as_.setAborted(result_);
        }
        if (!succeeded && !preempted && reached)
        {
            succeeded = true;
            result_.reached_pose = true;
            as_.setSucceeded(result_);

            ROS_INFO("[GoToPoseServer] Reached wanted pose: SUCCEEDDED!");
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