#ifndef ALPHA_SIMULATOR_SIMULATOR_H
#define ALPHA_SIMULATOR_SIMULATOR_H

#include "state.hxx"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "alpha_msgs/ThrustSignal.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Float64.h"
#include "chrono"
#include "thread"

class Simulator {
private:

    double m_simulated_depth;

    double m_dt; // seconds

    void iterate(control_commands_t cmd);

    void loop();

    void publish_odometry();

    void publish_diagnostics();

    void cmd_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

    void main_thruster_cb(const std_msgs::Float64::ConstPtr& msg);

    void horizontal_thruster_cb(const std_msgs::Float64::ConstPtr& msg);

    void vertical_thruster_cb(const std_msgs::Float64::ConstPtr& msg);

    ros::NodeHandle m_nh;

    ros::NodeHandle m_pnh;

    ros::Publisher m_odom_publisher;

    ros::Publisher m_diagnostic_publisher;

    ros::Subscriber m_cmd_subscriber;

    ros::Subscriber m_main_thruster_setpoint;

    ros::Subscriber m_horizontal_thruster_setpoint;

    ros::Subscriber m_vertical_thruster_setpoint;

    std::thread m_loop_thread;

    std::thread m_100hz_thread;

    std::thread m_10hz_thread;

    std::string m_tf_prefix;

public:
    Simulator();

};


#endif //ALPHA_SIMULATOR_SIMULATOR_H
