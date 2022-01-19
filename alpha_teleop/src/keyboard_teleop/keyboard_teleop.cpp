/*
    Program components:
     - keyboard_teleop.h
     - <keyboard_teleop.cpp>
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
#include "keyboard_teleop.h"



//clamping the thrusters to their respective min/max values, used later.
void clamp_t::main(double &val) {
    if (abs(val) > main_max) {
        (val > 0) ? val = main_max : val = -main_max;
    }
}
void clamp_t::horiz(double &val) {
    if (abs(val) > horiz_max) {
        (val > 0) ? val = horiz_max : val = -horiz_max;
    }
}
void clamp_t::vert(double &val) {
    if (abs(val) > vert_max) {
        (val > 0) ? val = vert_max : val = -vert_max;
    }
}

void KeyboardTeleop::f_setup_node() {
    //This is required to activate ncurses:
    //Initialize the screen, make it so keys are captured and not echoed to the output, and configure the terminal
    m_key_teleop_main_publisher = m_nh.advertise<std_msgs::Float64>(m_topic_main, 1);
    m_key_teleop_vertical_publisher = m_nh.advertise<std_msgs::Float64>(m_topic_vertical, 1);
    m_key_teleop_horizontal_publisher = m_nh.advertise<std_msgs::Float64>(m_topic_horizontal, 1);
    initscr();
    noecho();
    cbreak();
    keypad(stdscr, TRUE);
    start_color();
    notimeout(stdscr, TRUE);
    init_pair(1, COLOR_YELLOW, COLOR_GREEN);
    init_pair(2, COLOR_CYAN, COLOR_BLUE);
    init_pair(3, COLOR_BLACK, COLOR_WHITE);
    init_pair(4, COLOR_RED, COLOR_MAGENTA);        
    int h, w;
    getmaxyx(stdscr, h, w);
}

void KeyboardTeleop::f_step() {
    //getch() is an ncurses function which is supposed to be non-blocking. It used to have this behavior for me, but it currently is not working on my setup. 
    m_current_keypress = getch();
    if(m_current_keypress == ERR ) {
        return;
    } else {
        wrefresh(stdscr);
        switch(m_current_keypress) {
            case keys::w : {
                //Add to the thrusters output value, and clamp the value between upper and lower bounds defined in clamp_t. 
                m_main_thruster_out+=m_main_thruster_step_size;
                clamp.main(m_main_thruster_out);
                m_main_msg.data = m_main_thruster_out;
                break;
            }
            case keys::s : {
                m_main_thruster_out-=m_main_thruster_step_size;
                clamp.main(m_main_thruster_out);
                m_main_msg.data = m_main_thruster_out;
                break;
            }
            case keys::a: {
                m_horiz_thruster_out-=m_horiz_thruster_step_size;
                clamp.horiz(m_horiz_thruster_out);
                m_horiz_msg.data = m_horiz_thruster_out;
                break;
            }
            case keys::d: {
                m_horiz_thruster_out+=m_horiz_thruster_step_size;
                clamp.horiz(m_horiz_thruster_out);
                m_horiz_msg.data = m_horiz_thruster_out;
                break;
            }
            //If the user is holding shift, you set the max speed
            case keys::W : {
                m_main_msg.data = 1;
                break;
            }
            case keys::S : {
                m_main_msg.data = -1;
                break;
            }
            case keys::A: {
                m_horiz_msg.data = -1;
                break;
            }
            case keys::D: {
                m_horiz_msg.data = 1;
                break;
            }
            //Arrow keys
            case keys::UP_KEY: {
                m_vert_thruster_out+=m_vert_thruster_step_size;
                clamp.vert(m_vert_thruster_out);
                m_vert_msg.data = m_vert_thruster_out;
                break;
            }
            case keys::DOWN_KEY: {
                m_vert_thruster_out-=m_vert_thruster_step_size;
                clamp.vert(m_vert_thruster_out);
                m_vert_msg.data = m_vert_thruster_out;
                break;
            }
            //If colon is pressed, it escapes the loop and lets you provide more intricate messages. Similar to vim. 
            case keys::COLON: {
                echo();
                std::cout << "\n\r";
                char buffer[64];
                getstr(buffer);
                m_usr_message = static_cast<std::string>(buffer);
                std::cout << ":" << m_usr_message << "\n\r\r";
                std::string type, state;
                int sep = -1;
                sep = m_usr_message.find(" ");
                type = m_usr_message.substr(0,sep);
                state = m_usr_message.substr(sep+1,m_usr_message.length()-sep);

                //Example command you can put in
                if(type == "example_type") {
                    if(state == "0") {
                        std::cout << "\rzero\r\n\r\r";
                    }
                    if(state == "1") {
                        std::cout << "\rone\r\n\r\r";
                    }
                } else if(false) {
                    //something else can go here
                } else {
                    std::cout << "Command not recognized..\n\r";
                }
                noecho();
                break;
            }
            default: {
                //Leave for future debug/key additions
                //std::cout << "Number input.. " << m_current_keypress << "\n\r";
                //std::cout << "Char input.. " << (char)m_current_keypress << "\n\r";
                //std::cout << (char)m_current_keypress << "\n\r";
                break;
            }
        }
    }
}
void KeyboardTeleop::f_publish_msgs() {
    m_key_teleop_main_publisher.publish(m_main_msg);
    m_key_teleop_horizontal_publisher.publish(m_horiz_msg);
    m_key_teleop_vertical_publisher.publish(m_vert_msg);
    ROS_INFO("\rMAIN: %f",m_main_msg.data);
    ROS_INFO("\rHORIZ: %f",m_horiz_msg.data);
    ROS_INFO("\rVERT: %f",m_vert_msg.data);
}

KeyboardTeleop::KeyboardTeleop():
    m_nh(),
    m_pnh("~") {
    f_setup_node();
}
        
void KeyboardTeleop::run() {
    ros::Rate rate(20);
    while(true) {
        this->f_step();
        this->f_publish_msgs();
        rate.sleep();
        ros::spinOnce();
    }
}