#include "control_arm/gotopose_server.h"
#include "schunk_lwa4p_control/GoToPoseAction.h"


GoToPoseActionServer::GoToPoseActionServer(std::string name) :
    as_(nh_, name, boost::bind(&GoToPoseActionServer::executeCB, this, _1), false),
    action_name_(name)
    {
        as_.start();
    }

GoToPoseActionServer::~GoToPoseActionServer(void)
{

}

void executeCB(const schunk_lwa4p_control::GoToPoseGoalConstPtr &goal)
{
    //helper variables
    ros::Rate r(1);
    bool success = true;

    // set data to feedback

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "go_to_pose");

    GoToPoseAction gotoposeaction("go_to_pose");
    ros::spin();

    return 0;
}