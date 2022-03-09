#include "alpha_control/alpha_control_ros.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "alpha_control");

    AlphaControlROS control_ros;

    control_ros.initialize();

    ros::spin();

    return 0;
}