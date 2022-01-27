#include "ros/ros.h"

#include "alpha_driver/AlphaDriverRos.h"

int main(int argc, char** argv) {

    ros::init(argc, argv,"alpha_driver");

    AlphaDriverRos node;

    node.initialize();

    ros::spin();

    return 0;
}