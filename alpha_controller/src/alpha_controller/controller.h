#ifndef ALPHA_CONTROLLER_CONTROLLER_H
#define ALPHA_CONTROLLER_CONTROLLER_H

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "memory"

class Controller {
private:

    float m_desired_depth;
    float m_desired_heading;

    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;

    ros::Subscriber m_odom_sub;
    ros::Publisher m_thrust_pub;

    nav_msgs::Odometry m_odom;
    void odom_callback(const nav_msgs::Odometry::ConstPtr& msg);

    void iterate();

public:
    Controller();

};


#endif //ALPHA_CONTROLLER_CONTROLLER_H
