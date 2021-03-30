#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <schunk_lwa4p_control/GoToPoseAction.h>


class GoToPoseActionServer{
    protected:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<schunk_lwa4p_control::GoToPoseAction> as_;
        std::string action_name_;
        schunk_lwa4p_control::GoToPoseFeedback feedback_;
        schunk_lwa4p_control::GoToPoseResult result_;

    public:

    GoToPoseActionServer(std::string name) :
        as_(nh_, name, boost::bind(&GoToPoseActionServer::executeCB, this, _1), false),
        action_name_(name)
    {
        as_.start();
    }

    ~GoToPoseActionServer(void)
    {

    }

    void initializeSubscribers()
    {
        // Init subscriber for current state (arm)
    }

    void executeCB(const schunk_lwa4p_control::GoToPoseGoalConstPtr &goal)
    {
        ros::Rate(1);
        bool success = false;
    }


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gotopose");

    GoToPoseActionServer gotopose("go_to_pose");
    ros::spin();
    return 0;

}