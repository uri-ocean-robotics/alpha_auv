#include "keyboard_teleop/keyboard_teleop.hpp"

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "keyboard_teleop_node");

    KeyboardTeleop keyboard_teleop_rosnode;

    keyboard_teleop_rosnode.run();

    return 0;
}