#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <thread>
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
#include <moveit_msgs/PositionIKRequest.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/ChangeDriftDimensions.h>

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

        ros::Publisher targetPosePublisher;
        ros::Publisher currentPoseErrorPublisher;
        ros::Publisher magneticNavigationPublisher;
        ros::Subscriber currentPoseSubscriber;

        // service clients
        ros::ServiceClient startJointGroupPositionControllerClient_;
        ros::ServiceClient startJointGroupVelocityControllerClient_;
        ros::ServiceClient checkIKSolutionsClient_;
        ros::ServiceClient changeDriftDimensionsClient_;

        // action
        schunk_lwa4p_control::ServoTrackPoseFeedback feedback_;
        schunk_lwa4p_control::ServoTrackPoseResult result_;
        schunk_lwa4p_control::ServoTrackPoseGoal goal_;

        // msgs
        geometry_msgs::Pose currentPose;
        geometry_msgs::Pose cmdPose;

        // transformListener
        tf::TransformListener listener;

        // planningScene
        planning_scene_monitor::PlanningSceneMonitorPtr planningSceneMonitor_;
        bool pscLoaded = false;

        // thread
        std::thread move_to_pose_thread;

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

        // Add servo_nh_

        initializeSubscribers();
        initializePublishers();
        initializeServices();

        as_.start();
        ROS_INFO("[ServoTrackPoseServer] Starting...");
    }

    ~ServoTrackPoseServer(void)
    {
        // TODO: Check how looks descrutor of typical action_server
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

    void initializeSubscribers()
    {
        currentPoseSubscriber = as_nh_.subscribe<geometry_msgs::Pose>("/control_arm_node/tool/current_pose", 10, &ServoTrackPoseServer::currentPoseCB, this);
    }

    void initializePublishers()
    {
        targetPosePublisher = servo_nh_.advertise<geometry_msgs::PoseStamped>("/servo_server/target_pose", 1, true);
        currentPoseErrorPublisher = as_nh_.advertise<geometry_msgs::Point>("/pose_error", 1, true);
        magneticNavigationPublisher = as_nh_.advertise<geometry_msgs::PoseStamped>("/magnetic_estimation", 1, true);
    }

    void initializeServices()
    {
        ros::NodeHandle empty_nh_("");
        // Not Good! Node name for control_arm_node is hardcoded, have that in mind
        startJointGroupPositionControllerClient_ = empty_nh_.serviceClient<std_srvs::Trigger>("/control_arm_node/controllers/start_joint_group_position_controller");
        startJointGroupPositionControllerClient_.waitForExistence();
        startJointGroupVelocityControllerClient_ = empty_nh_.serviceClient<std_srvs::Trigger>("/control_arm_node/controllers/start_joint_group_velocity_controller");
        startJointGroupVelocityControllerClient_.waitForExistence();
        //checkIKSolutionsClient_ = empty_nh_.serviceClient<moveit_msgs::GetPositionIK>("/control_arm_node/arm/check_ik_solutions");
        //checkIKSolutionsClient_.waitForExistence();
        changeDriftDimensionsClient_ = empty_nh_.serviceClient<moveit_msgs::ChangeDriftDimensions>("/servo_server/change_drift_dimensions");
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

        // Check quaternion difference from moveit_servo!
        float x_ang_dist = 0; //pow((pose1.orientation.x - pose2.orientation.x), 1/2); // Not really useful
        float y_ang_dist = 0; //pow((pose1.orientation.y - pose2.orientation.y), 1/2);
        float z_ang_dist = 0; //pow((pose1.orientation.z - pose2.orientation.z), 1/2);
        float w_ang_dist = 0; //pow((pose1.orientation.w - pose2.orientation.w), 1/2);

        float dist = sqrt(x_dist + y_dist + z_dist + x_ang_dist + y_ang_dist + z_ang_dist + w_ang_dist);

        return dist;
    }

    float checkError(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)

    {
        float x_err = pose1.position.x - pose2.position.x;
        float y_err = pose1.position.y - pose2.position.y;
        float z_err = pose1.position.z - pose2.position.z;

        // Added it to topic for publishing
        //ROS_INFO_STREAM("x_err: " << x_err << "y_err:" << y_err << "z_err" << z_err);
        geometry_msgs::Point errorPoseMsg;
        errorPoseMsg.x = x_err; errorPoseMsg.y = y_err; errorPoseMsg.z = z_err;
        currentPoseErrorPublisher.publish(errorPoseMsg);

        float max_err_per_axis = std::max({std::abs(x_err), std::abs(y_err), std::abs(z_err)});

        return max_err_per_axis;

    }

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

    bool signum(float x)
    {
        if (x > 0) return 1;
        if (x < 0) return -1;
        return 0;
    }

    float limit_reference(float current_err, float max_ref, float min_error){

        if (abs(current_err) > min_error){
            if (abs(current_err) > max_ref){
                if(signum(current_err) == 1){
                    return max_ref;
                }
                if(signum(current_err) == -1){
                    return -max_ref;
                }
            }
        }else{
            return 0;
        }

    }

    // TODO: Fix transition between state 2 and 3 -- ??
    void executeCB(const schunk_lwa4p_control::ServoTrackPoseGoalConstPtr &goal)
    {
        ros::Rate r(1);
        ROS_INFO("[ServoTrackPoseServer] Recieved new goal!");

        ros::NodeHandle servo_nh_("servo_server");

        // It subscribes to wrong topic probably
        moveit_servo::PoseTracking tracker(servo_nh_, planningSceneMonitor_);

        StatusMonitor status_monitor(servo_nh_, "status");

        // AS flags
        bool elapsed = false; // execution timeout flag
        bool reached = false; // reached wanted pose with servoing
        bool preempted = false; // as has been preempted
        int goal_recieved_t = ros::Time::now().toSec();

        // Goal variables
        cmdPose = static_cast<geometry_msgs::Pose>(goal->final_pose);
        // Epsilon
        float epsilon = goal->max_err_per_axis;
        // Setup tracker_rate && check if estimated_duration > timeout to abort;
        int tracker_freq = goal->cmd_freq;
        if (tracker_freq == 0) tracker_freq = 100; ros::Rate tracker_rate(tracker_freq);
        // Setup timeout to 30 sec if not given
        int timeout = goal->timeout_sec;
        if (timeout == 0) timeout=30;

        // PoseTracking tolerances./sc
        Eigen::Vector3d lin_tol {0.005, 0.005, 0.005}; double rot_tol = 2; // Add this to goal if neccessary

        // Start JointGroupPositionController --> start JointGroupPositionController for servoing
        std_srvs::Trigger srv;
        startJointGroupPositionControllerClient_.call(srv);
        ROS_INFO("[ServoTrackPoseServer] Starting joint group position controller.");

        r.sleep();

        // Get current ee pose
        geometry_msgs::TransformStamped current_ee_tf;
        tracker.getCommandFrameTransform(current_ee_tf);

        tracker.resetTargetPose();

        // Create target pose
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.stamp = ros::Time::now();
        target_pose.header.frame_id = current_ee_tf.header.frame_id;
        target_pose.pose.position.x = current_ee_tf.transform.translation.x;
        target_pose.pose.position.y = current_ee_tf.transform.translation.y;
        target_pose.pose.position.z = current_ee_tf.transform.translation.z;
        target_pose.pose.orientation.x = current_ee_tf.transform.rotation.x;
        target_pose.pose.orientation.y = current_ee_tf.transform.rotation.y;
        target_pose.pose.orientation.z = current_ee_tf.transform.rotation.z;
        target_pose.pose.orientation.w = current_ee_tf.transform.rotation.w;

        ROS_INFO_STREAM("Enabling drift dimensions. ");
        moveit_msgs::ChangeDriftDimensions cdd_req;
        cdd_req.request.drift_y_translation = true;
        cdd_req.request.drift_x_translation = false;
        changeDriftDimensionsClient_.call(cdd_req);

        // Check existence of IK for wanted pose --> if exists continue, else break;
        moveit_msgs::GetPositionIK ik_req;
        ik_req.request.ik_request.pose_stamped = target_pose;
        checkIKSolutionsClient_.call(ik_req);
        ROS_INFO_STREAM("Checking IK!");

        std::thread move_to_pose_thread(
                [&tracker, &lin_tol, &rot_tol] { tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */);
                });

        // Added reached instead of check distance
        while (!reached && !elapsed && !preempted) { // !done --> used in for loop to terminate for loop

            // Publish estimated pose from magnetic navigation
            geometry_msgs::PoseStamped magnetic_pose;
            if(goal->magnetic_navigation == true){
                tf::StampedTransform transform1; tf::StampedTransform transform2;

                magnetic_pose.header.frame_id="magnetic_field_dest";
                listener.lookupTransform("base_link", "power_line0", ros::Time(0), transform1);
                listener.lookupTransform("base_link", "power_line1", ros::Time(0), transform2);

                magnetic_pose.pose.position.x = (transform1.getOrigin().x() + transform2.getOrigin().x())/2;
                magnetic_pose.pose.position.y = (transform1.getOrigin().y() + transform2.getOrigin().y())/2;
                magnetic_pose.pose.position.z = (transform1.getOrigin().z() + transform2.getOrigin().z())/2;
                magnetic_pose.pose.orientation.x = cmdPose.orientation.x; //transform1.getRotation().x();
                magnetic_pose.pose.orientation.y = cmdPose.orientation.y; //transform1.getRotation().y();
                magnetic_pose.pose.orientation.z = cmdPose.orientation.z; //transform1.getRotation().z();
                magnetic_pose.pose.orientation.w = cmdPose.orientation.w; //transform1.getRotation().w();
                //ROS_INFO_STREAM("x_est: " << magnetic_pose.pose.position.x << " x_real: " << target_pose.pose.position.x);
                //ROS_INFO_STREAM("y_est: " << magnetic_pose.pose.position.y << " y_real: " << target_pose.pose.position.y);
                //ROS_INFO_STREAM("z_est: " << magnetic_pose.pose.position.z << " z_real: " << target_pose.pose.position.z);
                magneticNavigationPublisher.publish(magnetic_pose);
                targetPosePublisher.publish(target_pose);

            }

            //ROS_INFO("[ServoTrackPoseServer] Currently executing...");
            feedback_.current_pose.position = currentPose.position;
            feedback_.current_pose.orientation = currentPose.orientation;
            as_.publishFeedback(feedback_);

            target_pose.header.stamp = ros::Time::now();

            target_pose.pose.position.x = cmdPose.position.x;
            target_pose.pose.position.y = cmdPose.position.y;
            target_pose.pose.position.z = cmdPose.position.z;
            //target_pose.pose.position.x = magnetic_pose.pose.position.x;
            //target_pose.pose.position.y = magnetic_pose.pose.position.y;
            //target_pose.pose.position.z = magnetic_pose.pose.position.z;
            // Keep same orientation --> align with wires
            target_pose.pose.orientation.x = cmdPose.orientation.x;
            target_pose.pose.orientation.y = cmdPose.orientation.y;
            target_pose.pose.orientation.z = cmdPose.orientation.z;
            target_pose.pose.orientation.w = cmdPose.orientation.w;

            targetPosePublisher.publish(target_pose);

            tracker_rate.sleep();   //TODO: Check what happens if tracker rate sleep is removed

            // Break loop if timeout elapsed
            elapsed = ( ros::Time::now().toSec() - goal_recieved_t) > timeout; // Check why this doesn't work

            if (elapsed) break;

            if ( as_.isPreemptRequested() || !ros::ok() )
            {
                ROS_INFO("[ServoTrackPoseServer] Preempted");
                // set the action state to preempted
                preempted = true;
                elapsed = false;
                tracker.stopMotion();
                move_to_pose_thread.join();
                ROS_INFO("[ServoTrackPoseServer] Preemption succeeded. ");
                as_.setPreempted();
                break;
            }

            float euclideanDistance = checkDist(currentPose, cmdPose);
            float orientationDist = checkOrientationDist(currentPose, cmdPose);
            float maxError = checkError(currentPose, cmdPose);

            // ROS_INFO_STREAM("Current servo error is: " << maxError);

            // This should break him if position has been reached
            if (maxError < epsilon and orientationDist < epsilon){
                reached = true;
                result_.reached_pose = true;
                break;
            }
        }

        if (!preempted){
            // causes std::system_error
            tracker.stopMotion();
            move_to_pose_thread.join();
        }

        // Set as to preempted --> This executable condition needs revision
        if (!preempted && elapsed)
        {
            ROS_INFO("[ServoTrackPoseServer] Timeout reached: ABORTED");
            as_.setAborted(result_);

        }

        // Set as to succeeded
        if(!preempted && reached)
        {
            result_.reached_pose = true;
            as_.setSucceeded(result_);
            ROS_INFO("[ServoTrackPoseServer] Reached wanted pose: SUCCEEDDED!");
        }

        // ros::Rate sleep_rate(1);
        // sleep_rate.sleep();

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