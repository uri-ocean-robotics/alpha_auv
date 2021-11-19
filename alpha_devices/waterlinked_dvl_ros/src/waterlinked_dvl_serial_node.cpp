#include "waterlinked_dvl/waterlinked_dvl_serial.h"

int main(int argc, char* argv[]) {

    ros::init(argc, argv, "waterlinked_dvl");

    WaterlinkedDvlSerial w;

    ros::spin();

    return 0;
}
