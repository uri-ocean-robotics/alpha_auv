#ifndef ALPHA_DRIVER_ALPHADRIVERROS_H
#define ALPHA_DRIVER_ALPHADRIVERROS_H

#include "AlphaDriver.h"

#include "alpha_msgs/NMEA.h"
#include "alpha_msgs/Pressure.h"
#include "alpha_msgs/Power.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "boost/shared_ptr.hpp"
#include "geometry_msgs/Vector3Stamped.h"
#include "map"
#include "exception"


#define STATIC_STRING static constexpr const char*


namespace DriverDict {
    STATIC_STRING CONF_THRUSTERS = "thrusters";
    STATIC_STRING CONF_THRUSTERS_PWM_CHANNEL = "pwm_channel";
    STATIC_STRING CONF_THRUSTERS_TOPIC = "topic";
}

class alpha_driver_ros_exception : public std::runtime_error {
public:
    explicit alpha_driver_ros_exception(const std::string& message) : std::runtime_error(message){}
};

class AlphaDriverRos {
private:

    typedef struct thruster_t {
        int pwm_channel;
        std::string topic;
        std::string name;
    } thruster_t;

    std::map<std::string, AlphaDriverRos::thruster_t> m_thrusters;

    std::vector<ros::Subscriber> m_thrust_cmd_subscribers;

    std::vector<double> m_thrust_per_channel;

    ros::NodeHandle m_nh;

    ros::NodeHandle m_pnh;

    ros::Subscriber m_raw_nmea_sub;

    ros::Subscriber m_struct_nmea_sub;

    ros::Publisher m_raw_nmea_pub;

    ros::Publisher m_struct_nmea_pub;

    ros::Publisher m_pressure_pub;

    ros::Publisher m_power_pub;

    ros::Publisher m_thrust_report_pub;

    std::string m_port;

    double m_thrust_main;

    double m_thrust_vertical;

    double m_thrust_horizontal;

    double m_security_timeout;

    int m_baud;

    std::shared_ptr<AlphaDriver> m_driver;

    ros::Time m_last_thrust_command_time;

    void f_raw_nmea_callback(const std_msgs::String::ConstPtr& msg);

    void f_struct_nmea_callback(const alpha_msgs::NMEA::ConstPtr& msg);

    void f_thrust_cb(const std_msgs::Float64::ConstPtr &msg, uint8_t channel);

    void f_driver_serial_callback(std::string incoming);

    void f_command_thrust_loop();

    void f_generate_thrusters();

public:
    AlphaDriverRos();

    void initialize();

};


#endif //ALPHA_DRIVER_ALPHADRIVERROS_H
