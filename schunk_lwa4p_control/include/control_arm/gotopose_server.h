#ifndef SCHUNK_LWA4P_CONTROL_GOTOPOSE_SERVER_H
#define SCHUNK_LWA4P_CONTROL_GOTOPOSE_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <schunk_lwa4p_control/GoToPoseAction.h>

class GoToPoseActionServer
{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<schunk_lwa4p_control::GoToPoseAction> as_;
        std::string action_name_;
        // Create messages that are used to publish feedback result;
        schunk_lwa4p_control::GoToPoseFeedback feedback_;
        schunk_lwa4p_control::GoToPoseResult result_;

    public:
        GoToPoseActionServer(std::string name);
        ~GoToPoseActionServer(void);
        void executeCB(const schunk_lwa4p_control::GoToPoseGoalConstPtr &goal);
};

#endif //SCHUNK_LWA4P_CONTROL_GOTOPOSE_SERVER_H
