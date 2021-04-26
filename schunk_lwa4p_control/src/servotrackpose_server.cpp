#include <ros/ros.h>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/Trigger.h>
// servo stuff
#include <std_msgs/Int8.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>
// action specific stuff
#include "schunk_lwa4p_control/ServoTrackPoseAction.h"

static const std::string LOGNAME= "servo_track_pose_server"

// Class for monitoring status of moveit servo

class StatusMonitor
{
public:
    StatusMonitor(ros::NodeHandle & nh, const std::string& topic)
    {
        sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
    }
private:
    void statusCB(const std_msgs::Int8ConstPtr& msg)
    {
        moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
        if (latest_status != status_)
        {
            status_ = latest_status;
            const auto &status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
            ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status" << status_str);
        }
    }

    moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
    ros::Subscriber sub_;
};


class ServoTrackPoseServer{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<schunk_lwa4p_control::GoToPoseAction> as_;
        std::string action_name_;

        // publishers
        ros::Publisher twistStampedPublisher;
        ros::Publisher jointServoPublisher;
        ros::Publisher targetPosePublisher;

        // subscribers
        ros::Subscriber currentPoseSubscriber;

        // service clients
        ros::ServiceClient realRobotDriverInitServiceClient_;

        // action
        schunk_lwa4p_control::ServoTrackPoseServerFeedback feedback_;
        schunk_lwa4p_control::ServoTrackPoseServerResult result_;
        schunk_lwa4p_control::ServoTrackPoseGoal goal_;

        // msgs
        geometry_msgs::Pose currentPose;
        geometry_msgs::Pose cmdPose;

    public:

    ServoTrackPoseServer(std::string name) :
        as_(nh_, name, boost::bind(&ServoTrackPoseServer::executeCB, this, _1), false),
        action_name_(name)
    {

        pscLoaded = loadPlanningSceneMonitor();
        if (pscLoaded)
        {
            initializeSubscribers();
            initializePublishers();
        }

        //startServo();
        startTracker();
        as_.start();

    }

    ~ServoTrackPoseServer(void)
    {
        servo.setPaused(true);
    }


    bool loadPlanningSceneMonitor()
    {
        planning_scene_monitor::PlanningSceneMonitorPtr planningSceneMonitor_;
        planningSceneMonitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
        if (!planningSceneMonitor_->getPlanningScene())
        {
            ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMontior");
            return false;
        }

        planningSceneMonitor_->startSceneMonitor();
        planningSceneMonitor_->startWorldGeometryMonitor(
                planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
                planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
                false /*skip octomap monitor*/);
        planningSceneMonior_->startStateMonitor();
        return true;
    }

    void startServo()
    {
        moveit_servo::Servo servo(nh, planningSceneMonitor_);
        servo.start();
    }

    void startTracker()
    {
        moveit_servo::PoseTracking tracker(nh, planningSceneMonitor_); // PoseTracker initializes servo
    }

    void initializeSubscribers()
    {
        currentPoseSubscriber = nh_.subscribe<geometry_msgs::Pose>("/control_arm_node/tool/current_pose", 10, &GoToPoseActionServer::currentPoseCB, this);

    }

    void initializePublishers()
    {
        twistStampedPublisher = nh_.advertise<geometry_msgs::TwistStamped>("/control_arm_node/arm/command/pose", 1);
        targetPosePublisher = nh_.advertise<geometry_msgs::PoseStamped>("/target_pose", 1, true);

    }

    void currentPoseCB(const geometry_msgs::Pose::ConstPtr &msg)
    {
        // https://answers.ros.org/question/212857/what-is-constptr/
        currentPose.position = msg->position;
        currentPose.orientation = msg->orientation;

    }


    float checkDist(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
    {
        float x_dist = pow((pose1.position.x - pose2.position.x), 2);
        float y_dist = pow((pose1.position.y - pose2.position.y), 2);
        float z_dist = pow((pose1.position.z - pose2.position.z), 2);

        float x_ang_dist = pow((pose1.orientation.x - pose2.orientation.x), 1/2);
        float y_ang_dist = pow((pose1.orientation.y - pose2.orientation.y), 1/2);
        float z_ang_dist = pow((pose1.orientation.z - pose2.orientation.z), 1/2);
        float w_ang_dist = pow((pose1.orientation.w - pose2.orientation.w), 1/2);

        float dist = sqrt(x_dist + y_dist + z_dist + x_ang_dist + y_ang_dist + z_ang_dist + w_ang_dist);

        return dist;
    }

    // TODO: Fix transition between state 2 and 3
    void executeCB(const schunk_lwa4p_control::GoToPoseGoalConstPtr &goal)
    {

        ros::Rate r(50);

        ROS_INFO_STREAM_NAMED(LOGNAME, "Received new goal!");

        bool sentCmd = false;
        bool elapsed = false;
        bool reached = false;
        bool preempted = false;
        bool succeeded = false;
        int tRecvGoal = ros::Time::now().toSec();

        // get Current pose
        // compare with final pose
        // publish target pose

        r.sleep();

        // Feedback publishing
        // feedback_.current_pose.position = currentPose.position;
        // feedback_.current_pose.orientation = currentPose.orientation;
        // as_.publishFeedback(feedback_);

        // Goal variables
        cmdPose = static_cast<geometry_msgs::Pose>(goal->goal_pose);
        float epsilon = goal->minimum_deviation;
        int timeout = goal->timeout_sec;

        // while not Final pose reached
        while (checkDist(cmdPose, currentPose) > epsilon && !elapsed && !preempted){
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

        // Should check for drifting (stop server if something drifts off, maybe difference between c
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