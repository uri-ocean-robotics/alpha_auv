#include "alpha_rf_teleop/alpha_rf_teleop.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "rf_node");

    RfTeleop r;

    ros::spin();

    return 0;
}