#include <ros/ros.h>
#include "control_arm/control_arm.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_arm");
    ros::NodeHandle nodeHandle("~"); 
    ControlArm controlArm(nodeHandle); 

    ros::spin();
    return 0; 

}