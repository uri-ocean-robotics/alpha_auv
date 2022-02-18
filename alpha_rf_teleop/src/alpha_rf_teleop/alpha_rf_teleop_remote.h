#ifndef ALPHA_RF_TELEOP_ALPHA_RF_TELEOP_REMOTE_H
#define ALPHA_RF_TELEOP_ALPHA_RF_TELEOP_REMOTE_H


#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "boost/asio/serial_port.hpp"
#include "boost/asio.hpp"
#include "boost/thread.hpp"
#include "geometry_msgs/PointStamped.h"
#include "rf_comms.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/String.h"
#include "seal_msgs/NMEA.h"
#include "geometry_msgs/Vector3Stamped.h"

#define NMEA_THRUST_PWM_CMD "PWMC"
#define NMEA_THRUST_PWM_REPORT "PWMR"
#define NMEA_BAROMETER_REPORT "BARR"
#define NMEA_MULTIMETER_REPORT "MULR"


class RfRemote{
private:

    boost::shared_ptr<RfComms> m_comms;

    std::string m_port;

    int m_baud;

    ros::NodeHandle m_nh;
    ros::NodeHandle m_pnh;

    ros::Subscriber m_joy_teleop_callback;

    ros::Publisher m_incoming_publisher;

    ros::Publisher m_gps_publisher;

    void f_joy_teleop_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

    void f_serial_callback(std::string incoming);

public:
    RfRemote();

};


#endif //ALPHA_RF_TELEOP_ALPHA_RF_TELEOP_REMOTE_H
