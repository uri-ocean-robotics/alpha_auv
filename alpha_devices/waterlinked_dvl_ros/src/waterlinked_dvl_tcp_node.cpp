#include "waterlinked_dvl/waterlinked_dvl_tcp.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "waterlinked_dvl");

    WaterlinkedDvlTcp w;

    ros::spin();

    return 0;
}
