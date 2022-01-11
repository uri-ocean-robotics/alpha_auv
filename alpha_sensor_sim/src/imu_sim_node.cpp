#include "imu_sim/imu_sim.hxx"

int main(int ac, char* av[]) {
    ros::init(ac, av, "imu_sim_node");

    imu_sim imu_sim_rosnode("GAUSSIAN_NOISE");

    while(ros::ok()) {
        imu_sim_rosnode.step();
        ros::spinOnce();
    }
}