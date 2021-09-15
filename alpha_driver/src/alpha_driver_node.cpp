#include "ros/ros.h"

#include "alpha_driver/AlphaRos.h"

int main(int argc, char** argv) {

    ros::init(argc, argv,"alpha_driver");

    AlphaRos node;

    ros::spin();

    return 0;
}