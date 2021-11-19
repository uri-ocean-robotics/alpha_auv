#include "alpha_controller/controller_mapper.h"


int main(int argc, char* argv[]) {

    ros::init(argc, argv, "controller_node");

    AlphaController c;

    ros::spin();

    return 0;
}
