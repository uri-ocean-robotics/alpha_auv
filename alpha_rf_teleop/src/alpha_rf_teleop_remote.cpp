#include "alpha_rf_teleop/alpha_rf_teleop_remote.h"

int main(int argc, char** argv) {

    ros::init(argc, argv, "rf_node");

    RfRemote r;

    ros::spin();

    return 0;
}