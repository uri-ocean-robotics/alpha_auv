#ifndef ALPHA_RF_TELEOP
#define ALPHA_RF_TELEOP

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "boost/asio/serial_port.hpp"
#include "boost/asio.hpp"
#include "boost/thread.hpp"
#include "geometry_msgs/PointStamped.h"
#include "rf_comms.h"
#include "sensor_msgs/NavSatFix.h"
#include "alpha_msgs/NMEA.h"
#include "std_msgs/String.h"

#define NMEA_THRUST_PWM_CMD "PWMC"
#define NMEA_THRUST_PWM_REPORT "PWMR"
#define NMEA_BAROMETER_REPORT "BARR"
#define NMEA_MULTIMETER_REPORT "MULR"


class RfTeleop{
private:

    boost::shared_ptr<RfComms> m_comms;

    std::string m_port;

    int m_baud;

    bool m_teleop_active;
    bool m_getty_active;

    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;

    ros::ServiceServer m_activate_getty_service;

    ros::ServiceServer m_activate_teleop_service;

    ros::Subscriber m_gps_callback;

    ros::Subscriber m_nmea_callback;

    ros::Publisher m_thrust_publisher;

    void f_gps_callback(const sensor_msgs::NavSatFix::ConstPtr& msg);

    void f_nmea_callback(const std_msgs::String::ConstPtr& msg);

    bool f_activate_getty(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    bool f_activate_teleop(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    void f_serial_callback(std::string incoming);

public:
    RfTeleop();

};

#endif