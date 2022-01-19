#include "dvl_sim/dvl_sim.h"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "dvl_sim_node");

    DvlSim dvl_sim_rosnode;

    dvl_sim_rosnode.run();

    return 0;
}