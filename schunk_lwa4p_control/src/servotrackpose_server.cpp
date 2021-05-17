#include <ros/ros.h>
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
        ros::Subscriber currentPoseSubscriber;

        // service clients
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
        initializeSubscribers();
        initializePublishers();
        initializeServices();
        //startServo();
        //moveit_servo::PoseTracking tracker(servo_nh_, planningSceneMonitor_);
        //startTracker(servo_nh_, planningSceneMonitor_);
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
    }

    void initializeServices()
    {
        ros::NodeHandle empty_nh_("");
        // Not Good! Node name for control_arm_node is hardcoded, have that in mind
        startJointGroupPositionControllerClient_ = empty_nh_.serviceClient<std_srvs::Trigger>("/control_arm_node/controllers/start_joint_group_position_controller");
        startJointGroupPositionControllerClient_.waitForExistence();

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

        ROS_INFO_STREAM("x_err: " << x_err << "y_err:" << y_err << "z_err" << z_err);

        float abs_err_sum = std::abs(x_err) + std::abs(y_err) + std::abs(z_err);

        return abs_err_sum/3;

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

    // TODO: Fix transition between state 2 and 3
    void executeCB(const schunk_lwa4p_control::ServoTrackPoseGoalConstPtr &goal)
    {
        ros::Rate r(1);
        ROS_INFO("[ServoTrackPoseServer] Recieved new goal!");

        // Start PoseTracking node upon recieving goal
        ros::NodeHandle servo_nh_("servo_server");
        ROS_INFO("[ServoTrackPoseServer] Initialized servo_nh");

        moveit_servo::PoseTracking tracker(servo_nh_, planningSceneMonitor_);
        StatusMonitor status_monitor(servo_nh_, "servo_status");
        bool elapsed = false; // execution timeout flag
        bool reached = false; // reached wanted pose with servoing
        bool preempted = false; // as has been preempted
        bool executable = true; // if execution_time > timeout -> false
        int tRecvGoal = ros::Time::now().toSec();

        // Goal variables
        cmdPose = static_cast<geometry_msgs::Pose>(goal->final_pose);

        float epsilon = goal->max_err_per_axis;
        int timeout = goal->timeout_sec;
        int tracker_freq = goal->cmd_freq;
        int n_segments = goal->n_segments;
        float mm_per_segment = goal->mm_per_segment/100; // mm -> to m
        std::string mode = goal->mode;
        Eigen::Vector3d lin_tol {0.005, 0.005, 0.005}; double rot_tol = 0.1; // Add this to goal if neccessary

        // Start JointGroupPositionController
        std_srvs::Trigger srv; startJointGroupPositionControllerClient_.call(srv);
        ROS_INFO("[ServoTrackPoseServer] Starting joint group position controller.");

        r.sleep();

        // More segments should result with smoother motion probably -> send it to method
        if (goal->n_segments != 0){
            n_segments = goal->n_segments;
        }else if(goal->mm_per_segment != 0){
            n_segments = checkError(cmdPose, currentPose)/mm_per_segment;
        }else{
            // Set default n_segments
            n_segments = 100;
        }

        ROS_INFO_STREAM("cmdPose is: " << cmdPose);
        //ROS_INFO_STREAM("currentPose is: " << currentPose);

        // TWO operating modes (static -> goal pose is fixed and increments are recalculated at every step)
        //                     (dynamic -> goal pose is variable)
        // defined in goal -> num_segments should be constant -> STATIC SERVO


        // Setup tracker_rate && check if estimated_duration > timeout to abort;
        if (tracker_freq == 0) tracker_freq = 50; ros::Rate tracker_rate(tracker_freq);
        // Setup timeout to 30 sec if not given
        if (timeout == 0) timeout=30;
        float estimated_duration = n_segments * tracker_rate.cycleTime().toSec();
        if(estimated_duration > timeout){
            executable = false;
            ROS_INFO("[ServoTrackPoseServer] Not executable, timeout. ");
        }

        // Get current ee pose
        geometry_msgs::TransformStamped current_ee_tf;
        tracker.getCommandFrameTransform(current_ee_tf);

        tracker.resetTargetPose();

        // Create target pose
        geometry_msgs::PoseStamped target_pose;
        target_pose.header.frame_id = current_ee_tf.header.frame_id;
        target_pose.pose.position.x = current_ee_tf.transform.translation.x;
        target_pose.pose.position.y = current_ee_tf.transform.translation.y;
        target_pose.pose.position.z = current_ee_tf.transform.translation.z;
        // Hardcode orientation for starters
        // target_pose.pose.orientation.x = 0; target_pose.pose.orientation.y = 0;
        // target_pose.pose.orientation.z = 0; target_pose.pose.orientation.w = 1;

        ROS_INFO_STREAM("Target pose translation is: " << current_ee_tf.transform.translation);
        ROS_INFO_STREAM("Target pose orientation is: " << current_ee_tf.transform.rotation);

        target_pose.header.stamp = ros::Time::now();
        targetPosePublisher.publish(target_pose);

        std::thread move_to_pose_thread(
                [&tracker, &lin_tol, &rot_tol] { tracker.moveToPose(lin_tol, rot_tol, 0.1 /* target pose timeout */);
                });

        // mode defines how we assign reference to pose tracking node
        bool done = false; bool failed = false; bool first_step = true;
        float x_increment; float y_increment; float z_increment;
        float initial_x_err; float initial_y_err; float initial_z_err;
        // Added reached instead of check distance
        while (!reached && !elapsed && !preempted && executable && !failed) { // !done --> used in for loop to terminate for loop

            feedback_.current_pose.position = currentPose.position;
            feedback_.current_pose.orientation = currentPose.orientation;
            as_.publishFeedback(feedback_);

            // No angle distance check, maybe use quaternions for that as stated in following link
            // https://math.stackexchange.com/questions/90081/quaternion-distance

            // Modify the pose target a little bit each cycle
            target_pose.header.stamp = ros::Time::now();
            target_pose.pose.position.x = cmdPose.position.x;
            target_pose.pose.position.y = cmdPose.position.y;
            target_pose.pose.position.z = cmdPose.position.z;
            // Keep same orientation
            target_pose.pose.orientation.x = cmdPose.orientation.x;
            target_pose.pose.orientation.y = cmdPose.orientation.y;
            target_pose.pose.orientation.z = cmdPose.orientation.z;
            target_pose.pose.orientation.w = cmdPose.orientation.w;

            targetPosePublisher.publish(target_pose);
            tracker_rate.sleep();

            // COMM
            // ROS_INFO_STREAM("cmdPose: " << cmdPose);
            // ROS_INFO_STREAM("currentPose " << currentPose);

            // float x_error = cmdPose.position.x - currentPose.position.x;
            // float y_error = cmdPose.position.y - currentPose.position.y;
            // float z_error = cmdPose.position.z - currentPose.position.z;

            // feedback_.current_pose.position = currentPose.position;
            // feedback_.current_pose.orientation = currentPose.orientation;
            // as_.publishFeedback(feedback_);

            // No angle distance check, maybe use quaternions for that as stated in following link
            // https://math.stackexchange.com/questions/90081/quaternion-distance
            // COMM

            // Modify the pose target a little bit each cycle | COMM
            // target_pose.pose.position.x = currentPose.position.x + x_increment;
            // target_pose.pose.position.y = currentPose.position.y + y_increment;
            // target_pose.pose.position.z = currentPose.position.z + z_increment;


            // Break loop if timeout elapsed
            elapsed = ( ros::Time::now().toSec() - tRecvGoal) > timeout;
            if (elapsed) break;

            if ( as_.isPreemptRequested() || !ros::ok() )
            {
                ROS_INFO("[ServoTrackPoseServer] Preempted");
                // set the action state to preempted
                as_.setPreempted();
                reached = false;
                preempted = true;
                elapsed = false;
                tracker.stopMotion();
                move_to_pose_thread.join();
            }

            float euclideanDistance = checkDist(currentPose, cmdPose);
            float orientationDist = checkOrientationDist(currentPose, cmdPose);
            float absError = checkError(currentPose, cmdPose);
            ROS_INFO_STREAM("current dist is: " << euclideanDistance);
            ROS_INFO_STREAM("current err is: " << absError);
            ROS_INFO_STREAM("epsilon is: " << epsilon);

            // This should break him if position has been reached
            if (absError < epsilon and orientationDist < epsilon){
                reached = true;
                result_.reached_pose = true;
                break;
            }

            done = true; // Flag for finishing loop
            /* for (size_t i = 0; i < n_segments; ++i)
            {
                float x_error = cmdPose.position.x - currentPose.position.x;
                float y_error = cmdPose.position.y - currentPose.position.y;
                float z_error = cmdPose.position.z - currentPose.position.z;
                // STATIC INCREMENT -> STATIC SERVO IF BEFORE FOR LOOP
                // DYNAMIC INCREMENT -> DEPENDING ON CURRENT STEP
                if (mode == std::string("dynamic")){
                    x_increment = x_error / (n_segments - i);
                    y_increment = y_error / (n_segments - i);
                    z_increment = z_error / (n_segments - i);
                }
                else if(mode == std::string("full")){
                    float max_reference = 0.05;
                    // reference limiting
                    x_increment = limit_reference(x_error, max_reference, 0.01); //x_error;
                    y_increment = limit_reference(y_error, max_reference, 0.01); //y_error;
                    z_increment = limit_reference(z_error, max_reference, 0.01); //z_error;

                }
                else{
                    float increment = 0.01;
                    float min_error = 0.01; float max_reference=0.01;
                    x_increment = x_error; //limit_reference(x_error, max_reference, 0.01);
                    y_increment = y_error; //limit_reference(y_error, max_reference, 0.01);
                    z_increment = z_error; //limit_reference(z_error, max_reference, 0.01);

                    // Minimal reference change
                    // x_increment = x_error / n_segments ; if (mm_per_segment != 0 && x_increment !=  mm_per_segment) x_increment = mm_per_segment;
                    // y_increment = y_error / n_segments ; if (mm_per_segment != 0 && y_increment !=  mm_per_segment) y_increment = mm_per_segment;
                    // z_increment = z_error / n_segments ; if (mm_per_segment != 0 && z_increment !=  mm_per_segment) z_increment = mm_per_segment;
                    // Minimal is 1mm increment to set up (controller resolution)
                }

                // Add error check to see if error increases to halt motion --> used for securing real robot from insane references
                if (first_step){
                    float initial_err_x = x_error; float initial_err_y = y_error; float initial_err_z = z_error;
                    first_step  = false;
                }
                float x_err_check = initial_x_err * 1.1; float y_err_check = initial_y_err * 1.1; float z_err_check = initial_z_err * 1.1;
                if (x_error > x_err_check || y_error > y_err_check || z_error >  z_err_check) failed = true;

                feedback_.current_pose.position = currentPose.position;
                feedback_.current_pose.orientation = currentPose.orientation;
                as_.publishFeedback(feedback_);

                // No angle distance check, maybe use quaternions for that as stated in following link
                // https://math.stackexchange.com/questions/90081/quaternion-distance

                // Modify the pose target a little bit each cycle
                target_pose.pose.position.x = currentPose.position.x + x_increment;
                target_pose.pose.position.y = currentPose.position.y + y_increment;
                target_pose.pose.position.z = currentPose.position.z + z_increment;
                // Keep same orientation
                target_pose.pose.orientation = currentPose.orientation;
                target_pose.header.stamp = ros::Time::now();

                ROS_INFO_STREAM("target x: " << target_pose.pose.position.x );
                ROS_INFO_STREAM("target y: " << target_pose.pose.position.y );
                ROS_INFO_STREAM("target z: " << target_pose.pose.position.z );
                //ROS_INFO_STREAM("target_pose_x")
                ROS_INFO_STREAM("z_increment: " << z_increment);
                targetPosePublisher.publish(target_pose);

                tracker_rate.sleep();

                // Break loop if timeout elapsed
                elapsed = ( ros::Time::now().toSec() - tRecvGoal) > timeout;
                if (elapsed) break;

                if ( as_.isPreemptRequested() || !ros::ok() )
                {
                    ROS_INFO("[ServoTrackPoseServer] Preempted");
                    // set the action state to preempted
                    as_.setPreempted();
                    reached = false;
                    preempted = true;
                    elapsed = false;
                    tracker.stopMotion();
                    move_to_pose_thread.join();
                }

                float euclideanDistance = checkDist(currentPose, cmdPose);
                float absError = checkError(currentPose, cmdPose);
                ROS_INFO_STREAM("current dist is: " << euclideanDistance);
                ROS_INFO_STREAM("current err is: " << absError);
                ROS_INFO_STREAM("epsilon is: " << epsilon);

                // This should break him if position has been reached
                if (absError < epsilon){
                    reached = true;
                    result_.reached_pose = true;
                    break;
                }

                done = true; // Flag for finishing loop
            }*/

            ROS_INFO_STREAM("elapsed: " << elapsed);
            ROS_INFO_STREAM("reached: " << reached);
            ROS_INFO_STREAM("preempted: " << preempted);
            ROS_INFO_STREAM("executable: " << executable);
        }

        tracker.stopMotion();
        move_to_pose_thread.join();

        // Set as to preempted
        if ((elapsed && !preempted) || !executable)
        {
            ROS_INFO("[ServoTrackPoseServer] Timeout reached: ABORTED");
            //tracker.stopMotion();
            //move_to_pose_thread.join();
            as_.setAborted(result_);
        }

        // Set as to succeeded
        if(!preempted && reached)
        {
            result_.reached_pose = true;
            as_.setSucceeded(result_);
            ROS_INFO("[ServoTrackPoseServer] Reached wanted pose: SUCCEEDDED!");
        }

        ros::Rate sleep_rate(1);
        sleep_rate.sleep();

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