
#include "iostream"
#include "alpha_simulator/simulator.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "alpha_simulator");

    Simulator s;

    ros::spin();

    return 0;
}