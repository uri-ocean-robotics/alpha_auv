#include "imu_sim/imu_sim.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "imu_sim_node");

    ImuSim imu_sim_rosnode;

    imu_sim_rosnode.run();

    return 0;
}