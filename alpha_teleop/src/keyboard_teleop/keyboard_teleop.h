/*
    Program components:
     - <keyboard_teleop.h>
     - keyboard_teleop.cpp
     - Part of the alpha_teleop package in ROS
    
    Author: 
     - Raymond Turrisi <raymond.turrisi@gmail.com>

    Organization: 
     - Smart Ocean Systems Lab

    Licence: 
     - MIT License

    Circa:
     - Spring 2022

    Description: 
     - Prototype for keyboard teleoperation for ALPHA AUV / ALPHA AUV SIM
     - It is currently meant to have intuitive controls. 
        w: main thruster forward
        s: horizontal thruster left
        a: main thruster reverse
        d: horizontal thruster right
        UP-ARROWKEY: vertical thruster up
        DOWN-ARROWKEY: vertical thruster down
*/

#pragma once 

#include <iostream>
#include <string>
#include <ncurses.h>
#include <chrono>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "ros/console.h"

#define CONST_INT constexpr uint16_t

//ncurses has globally scoped key codes for certain keys, but they are inconsistent. Here we add a dictionary for keycodes, which align with the ASCII table. 
namespace keys {
    CONST_INT w = 119;
    CONST_INT s = 115;
    CONST_INT a = 97;
    CONST_INT d = 100;
    CONST_INT W = 87;
    CONST_INT S = 83;
    CONST_INT A = 65;
    CONST_INT D = 68;
    CONST_INT DOWN_KEY = 258;
    CONST_INT UP_KEY = 259;
    CONST_INT LEFT_KEY = 260;
    CONST_INT RIGHT_KEY = 261;
    CONST_INT ONE = 49;
    CONST_INT TWO = 50;
    CONST_INT THREE = 51;
    CONST_INT FOUR = 52;
    CONST_INT ONE_SH = 33;
    CONST_INT TWO_SH = 64;
    CONST_INT THREE_SH = 35;
    CONST_INT FOUR_SH = 36;
    CONST_INT COLON = 58;
}


//clamping the thrusters to their respective min/max values, used later.
typedef struct clamp_t {
    double main_max = 1.0;
    double horiz_max = 1.0;
    double vert_max = 1.0;
    void main(double &val);
    void horiz(double &val);
    void vert(double &val);
} clamp_t;

class KeyboardTeleop {
    private:
        // I plan to make this more configurable/include parameters here
        int m_steps = 5;
        double m_main_thruster_out = 0.0;
        double m_horiz_thruster_out = 0.0;
        double m_vert_thruster_out = 0.0;
        double m_main_thruster_step_size = 1.0/m_steps;
        double m_horiz_thruster_step_size = 1.0/m_steps;
        double m_vert_thruster_step_size = 1.0/m_steps;
        double m_rate;
        
        clamp_t clamp;

        std_msgs::Float64 m_main_msg;
        std_msgs::Float64 m_horiz_msg;
        std_msgs::Float64 m_vert_msg;

        ros::NodeHandle m_nh;
        ros::NodeHandle m_pnh;

        static constexpr const char* m_topic_main = "control/thruster/main";

        static constexpr const char* m_topic_vertical = "control/thruster/vertical";

        static constexpr const char* m_topic_horizontal = "control/thruster/horizontal";

        ros::Publisher m_key_teleop_main_publisher;
        ros::Publisher m_key_teleop_vertical_publisher;
        ros::Publisher m_key_teleop_horizontal_publisher;

        void f_setup_node();

        int m_current_keypress;
        std::string m_usr_message;
        void f_step();
        void f_publish_msgs();

    public:
        KeyboardTeleop();
        
        void run();
};