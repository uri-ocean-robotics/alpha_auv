#ifndef ALPHA_SIMULATOR_SIMULATOR_H
#define ALPHA_SIMULATOR_SIMULATOR_H

#include "state.hxx"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "alpha_msgs/ThrustSignal.h"
#include "geometry_msgs/Vector3Stamped.h"
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

    ros::NodeHandle m_nh;

    ros::Publisher m_odom_publisher;

    ros::Publisher m_diagnostic_publisher;

    ros::Subscriber m_cmd_subscriber;

    std::thread m_loop_thread;

    std::thread m_100hz_thread;

    std::thread m_10hz_thread;

    std::string m_tf_prefix;

public:
    Simulator();

};


#endif //ALPHA_SIMULATOR_SIMULATOR_H
