#include <ros/ros.h>
#include "control_arm/control_arm.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_arm");
    ros::NodeHandle nodeHandle("~"); 
    ControlArm controlArm(nodeHandle); 

    int num_threads = 4;     
    ros::AsyncSpinner spinner(num_threads);
    spinner.start(); 
    controlArm.run(); 

    return 0; 

}