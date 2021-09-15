#ifndef ALPHA_DRIVER_ALPHAROS_H
#define ALPHA_DRIVER_ALPHAROS_H

#include "AlphaDriver.h"

#include "alpha_msgs/NMEA.h"
#include "alpha_msgs/Pressure.h"
#include "alpha_msgs/Power.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "boost/shared_ptr.hpp"
#include "geometry_msgs/Vector3Stamped.h"

class AlphaRos {
private:

    ros::NodeHandle nh_;

    ros::NodeHandle pnh_;

    ros::Subscriber raw_nmea_sub_;

    ros::Subscriber struct_nmea_sub_;

    ros::Publisher raw_nmea_pub_;

    ros::Publisher struct_nmea_pub_;

    ros::Publisher pressure_pub_;

    ros::Publisher power_pub_;

    ros::Publisher thrust_report_pub_;

    ros::Subscriber thrust_cmd_sub_;

    std::string port_;
    int baud_;

    void raw_nmea_callback(const std_msgs::String::ConstPtr& msg);

    void struct_nmea_callback(const alpha_msgs::NMEA::ConstPtr& msg);

    void thrust_cmd_callback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);

    void driver_serial_callback(std::string incoming);

    boost::shared_ptr<AlphaDriver> driver_;
public:
    AlphaRos();


};


#endif //ALPHA_DRIVER_ALPHAROS_H
