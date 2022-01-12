#include "imu_sim/imu_sim.h"

int main(int ac, char* av[]) {
    ros::init(ac, av, "imu_sim_node");

    imu_sim imu_sim_rosnode;
    ROS_INFO("STARTING IMU_SIM_NODE");

    while(ros::ok()) {
        imu_sim_rosnode.step();
        ros::spinOnce();
    }
}