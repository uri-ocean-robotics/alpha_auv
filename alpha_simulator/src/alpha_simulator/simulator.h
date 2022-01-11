#ifndef ALPHA_SIMULATOR_SIMULATOR_H
#define ALPHA_SIMULATOR_SIMULATOR_H

#include "state.hxx"

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "alpha_msgs/ThrustSignal.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/WrenchStamped.h"
#include "std_msgs/Float64.h"
#include "rosgraph_msgs/Clock.h"
#include "chrono"
#include "thread"

class Simulator {
private:

    double m_simulated_depth;

    uint64_t m_ts; // simulated time stamp

    double m_dt; // seconds

    void iterate(control_commands_t cmd);

    void loop();

    void publish_odometry();

    void publish_velocity();

    void publish_pose();

    void publish_acceleration();

    void publish_clock();

    void main_thruster_cb(const std_msgs::Float64::ConstPtr& msg);

    void horizontal_thruster_cb(const std_msgs::Float64::ConstPtr& msg);

    void vertical_thruster_cb(const std_msgs::Float64::ConstPtr& msg);

    ros::NodeHandle m_nh;

    ros::NodeHandle m_pnh;

    ros::Publisher m_odom_publisher;

    ros::Subscriber m_main_thruster_setpoint;

    ros::Subscriber m_horizontal_thruster_setpoint;

    ros::Subscriber m_vertical_thruster_setpoint;

    ros::Publisher m_acceleration_publisher;

    ros::Publisher m_velocity_publisher;

    ros::Publisher m_pose_publisher;

    ros::Publisher m_clock_publisher;

    std::thread m_loop_thread;


    std::thread m_100hz_thread;

    std::thread m_50hz_thread;

    std::thread m_10hz_thread;

    std::string m_tf_prefix;


public:
    Simulator();

};


#endif //ALPHA_SIMULATOR_SIMULATOR_H
