#include <ros/ros.h>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/Trigger.h>
// servo stuff
#include <std_msgs/Int8.h>
#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>
// action specific stuff
#include "schunk_lwa4p_control/ServoTrackPoseAction.h"

static const std::string LOGNAME= "servo_track_pose_server";

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
        ros::NodeHandle as_nh_;
        ros::NodeHandle servo_nh_;
        actionlib::SimpleActionServer<schunk_lwa4p_control::ServoTrackPoseAction> as_;
        std::string action_name_;

        // publishers
        ros::Publisher twistStampedPublisher;
        ros::Publisher jointServoPublisher;
        ros::Publisher targetPosePublisher;

        // subscribers
        ros::Subscriber currentPoseSubscriber;

        // service clients
        ros::ServiceClient realRobotDriverInitServiceClient_;
        ros::ServiceClient startJointGroupPositionControllerClient_;

        // action
        schunk_lwa4p_control::ServoTrackPoseFeedback feedback_;
        schunk_lwa4p_control::ServoTrackPoseResult result_;
        schunk_lwa4p_control::ServoTrackPoseGoal goal_;

        // msgs
        geometry_msgs::Pose currentPose;
        geometry_msgs::Pose cmdPose;

        // planningScene
        planning_scene_monitor::PlanningSceneMonitorPtr planningSceneMonitor_;

        // pose tracking interface
        //moveit_servo::PoseTracking tracker;


        bool pscLoaded = false;


    public:

    ServoTrackPoseServer(std::string name) :
        as_(as_nh_, name, boost::bind(&ServoTrackPoseServer::executeCB, this, _1), false),
        action_name_(name)
    {
        while(!pscLoaded){

            ROS_INFO("[ServoTrackPoseServer] Waiting for planning scene...");
            pscLoaded = loadPlanningSceneMonitor();
            ros::Rate sleep_rate(1);
            sleep_rate.sleep();
        }
        initializeSubscribers();
        initializePublishers();
        //startServo();
        //moveit_servo::PoseTracking tracker(servo_nh_, planningSceneMonitor_);
        //startTracker(servo_nh_, planningSceneMonitor_);
        as_.start();
        ROS_INFO("[ServoTrackPoseServer] Starting...");


    }

    ~ServoTrackPoseServer(void)
    {
        //tracker.setPaused(true);
        //servo.setPaused;
    }


    bool loadPlanningSceneMonitor()
    {
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
        planningSceneMonitor_->startStateMonitor();
        return true;
    }

    void startServo(ros::NodeHandle nh, planning_scene_monitor::PlanningSceneMonitorPtr ps)
    {
        moveit_servo::Servo servo(nh, planningSceneMonitor_);
        servo.start();
    }

    //void startTracker(ros::NodeHandle nh, planning_scene_monitor::PlanningSceneMonitorPtr ps)
    //{
    //    moveit_servo::PoseTracking tracker(nh, ps); // PoseTracker initializes servo
    // check which nodes initialize tracker and under which name.
    // }

    void initializeSubscribers()
    {
        currentPoseSubscriber = as_nh_.subscribe<geometry_msgs::Pose>("/control_arm_node/tool/current_pose", 10, &ServoTrackPoseServer::currentPoseCB, this);

    }

    void initializePublishers()
    {
        targetPosePublisher = servo_nh_.advertise<geometry_msgs::PoseStamped>("/target_pose", 1, true);
        // namespace stuff?

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

    //float checkError(cmdPose, currentPose)
    //{
    //
    //}

    // TODO: Fix transition between state 2 and 3
    void executeCB(const schunk_lwa4p_control::ServoTrackPoseGoalConstPtr &goal)
    {

        ros::Rate r(50);

        ROS_INFO_STREAM_NAMED(LOGNAME, "Received new goal!");

        // Start PoseTracking node upon recieving goal
        ros::NodeHandle servo_nh_("servo_server");
        ROS_INFO("[ServoTrackPoseServer] Initialized servo_nh");

        moveit_servo::PoseTracking tracker(servo_nh_, planningSceneMonitor_);
        ROS_INFO("[ServoTrackPoseServer] Initialized tracker");
        StatusMonitor status_monitor(servo_nh_, "status");

        ROS_INFO("[ServoTrackPoseServer] Initialized StatusMonitor"); 

        bool sentCmd = false;
        bool elapsed = false;
        bool reached = false;
        bool preempted = false;
        bool succeeded = false;
        bool executable = true;
        int tRecvGoal = ros::Time::now().toSec();

        // Start JointGroupPositionControllers
        std_srvs::Trigger srv; startJointGroupPositionControllerClient_.call(srv);
        ROS_DEBUG_STREAM("[ServoTrackPoseServer] Starting joint group position controller.");

        r.sleep();

        // Feedback publishing
        // feedback_.current_pose.position = currentPose.position;
        // feedback_.current_pose.orientation = currentPose.orientation;
        // as_.publishFeedback(feedback_);

        // Goal variables
        cmdPose = static_cast<geometry_msgs::Pose>(goal->final_pose);
        float epsilon = goal->minimum_deviation;
        int timeout = goal->timeout_sec;
        int n_segments = 0;

        // More segments should result with smoother motion probably
        if (goal->n_segments != 0){
            n_segments = goal->n_segments;
        }else if(goal->euclid_per_segment != 0){
            n_segments = checkDist(cmdPose, currentPose)/goal->euclid_per_segment;
        }else{
            n_segments = 1000;
        }
        ROS_DEBUG_STREAM("[ServoTrackPoseServer] n_segments:" << n_segments);

        // TWO operating modes (static -> goal pose is fixed and increments are recalculated at every step)
        //                     (dynamic -> goal pose is variable)
        // defined in goal -> num_segments should be constant
        float x_error = cmdPose.position.x - currentPose.position.x;
        float y_error = cmdPose.position.y - currentPose.position.y;
        float z_error = cmdPose.position.z - currentPose.position.z;

        float x_increment = x_error / n_segments;
        float y_increment = y_error / n_segments;
        float z_increment = z_error / n_segments;

        float estimated_duration = n_segments * r.cycleTime().toSec();

        if(estimated_duration > timeout){
            executable = false;
        }

        Eigen::Vector3d lin_tol {0.01, 0.01, 0.01};
        double rot_tol = 0.1;

        // Get current ee pose
        geometry_msgs::TransformStamped current_ee_tf;
        tracker.getCommandFrameTransform(current_ee_tf);

        // Create target pose
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = current_ee_tf.header.frame_id;
        target_pose.pose.position.x = current_ee_tf.transform.translation.x;
        target_pose.pose.position.y = current_ee_tf.transform.translation.y;
        target_pose.pose.position.z = current_ee_tf.transform.translation.z;
        target_pose.pose.orientation = current_ee_tf.transform.rotation;

        tracker.resetTargetPose();

        target_pose.header.stamp = ros::Time::now();
        targetPosePublisher.publish(target_pose);

        std::thread move_to_pose_thread(
                [&tracker, &lin_tol, &rot_tol] {tracker.moveToPose(lin_tol, rot_tol, 0.1);});

        // while not Final pose reached
        while (checkDist(cmdPose, currentPose) > epsilon && !elapsed && !preempted && executable) {
            // Check timeout condition
            // publishes target pose based on some params :)
            //

            ros::Rate loop_rate(50);
            for (size_t i = 0; i < n_segments; ++i)
            {
                // Modify the pose target a little bit each cycle
                // This is a dynamic pose target
                target_pose.pose.position.z += z_increment;
                target_pose.pose.position.y += y_increment;
                target_pose.pose.position.x += x_increment;
                target_pose.header.stamp = ros::Time::now();
                targetPosePublisher.publish(target_pose);

                loop_rate.sleep();
                elapsed = ( ros::Time::now().toSec() - tRecvGoal) > timeout;

                if ( as_.isPreemptRequested() || !ros::ok() )
                {
                    ROS_INFO("[ServoTrackPoseServer] Preempted");
                    // set the action state to preempted
                    as_.setPreempted();
                    reached = false;
                    preempted = true;
                    elapsed = false;
                }

            }

            //TODO: Add condition check
            reached = true;

            //tracker.stopMotion();
            //move_to_pose_thread.join();


        }

        if (elapsed && !preempted && !executable)
        {
            ROS_INFO("[GoToPoseServer] Timeout reached: ABORTED");
            as_.setAborted(result_);
        }


    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "servotrackpose");
    ros::AsyncSpinner spinner(6);

    ServoTrackPoseServer servotrackpose("servo_track_pose");
    spinner.start();
    ros::waitForShutdown();
    return 0;

}