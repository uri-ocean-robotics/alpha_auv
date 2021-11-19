
#ifndef ALPHA_CONTROLLER_CONTROLLER_H
#define ALPHA_CONTROLLER_CONTROLLER_H

#include "ros/ros.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "cola2_msgs/Setpoints.h"

class AlphaController {
private:
    ros::NodeHandle m_pnh;
    ros::NodeHandle m_nh;

    ros::Subscriber m_cmd_vel_subscriber;

    ros::Subscriber m_set_point_subscriber;

    ros::Publisher m_cmd_pwm_publisher;

    void cmd_vel_callback(const geometry_msgs::Vector3Stamped& msg);

    void set_point_callback(const cola2_msgs::Setpoints& msg);

public:
    AlphaController();
};

#endif //ALPHA_CONTROLLER_CONTROLLER_H
