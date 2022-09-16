/*
    This file is part of ALPHA AUV project.

    This project is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This project is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with the project.  If not, see <https://www.gnu.org/licenses/>.

    Author: Emir Cem Gezer
    Email: emircem@uri.edu;emircem.gezer@gmail.com
    Year: 2022

    Copyright (C) 2022 Smart Ocean Systems Laboratory
*/

#ifndef ALPHA_DRIVER_ALPHADRIVERROS_H
#define ALPHA_DRIVER_ALPHADRIVERROS_H

#include "AlphaDriver.h"

#include "mvp_msgs/Float64Stamped.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "boost/shared_ptr.hpp"
#include "geometry_msgs/Vector3Stamped.h"
#include "map"
#include "exception"


#define STATIC_STRING static constexpr const char*


namespace DriverDict {
    STATIC_STRING CONF_PWM_CONTROL = "pwm_control";
    STATIC_STRING CONF_PWM_CHANNEL = "channel";
    STATIC_STRING CONF_PWM_MODE = "mode";
    STATIC_STRING CONF_PWM_MODE_OPT_THRUSTER = "thruster";
    STATIC_STRING CONF_PWM_MODE_OPT_PURE = "pure";
    STATIC_STRING CONF_PWM_TOPIC = "topic";
}

class alpha_driver_ros_exception : public std::runtime_error {
public:
    explicit alpha_driver_ros_exception(const std::string& message) : std::runtime_error(message){}
};

class AlphaDriverRos {
private:

    typedef struct pwm_control_t : pwm_t {
        std::string topic;
    } pwm_control_t;

    std::map<std::string, pwm_control_t> m_pwm_control;

    std::vector<ros::Subscriber> m_thrust_cmd_subscribers;

    std::vector<double> m_thrust_per_channel;

    ros::NodeHandle m_nh;

    ros::NodeHandle m_pnh;

    ros::Subscriber m_raw_nmea_sub;

    ros::Subscriber m_serial0_sub;

    ros::Subscriber m_serial1_sub;

    ros::Publisher m_serial0_pub;

    ros::Publisher m_serial1_pub;

    ros::Subscriber m_struct_nmea_sub;

    ros::Publisher m_raw_nmea_pub;

    ros::Publisher m_struct_nmea_pub;

    ros::Publisher m_pressure_pub;

    ros::Publisher m_depth_pub;

    ros::Publisher m_temperature_pub;

    ros::Publisher m_voltage_pub;

    ros::Publisher m_current_pub;

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

    void f_serial_callback(const std_msgs::String::ConstPtr& msg, int channel);

    void f_pwm_cb(const std_msgs::Float64::ConstPtr &msg, uint16_t channel, uint8_t mode);

    void f_driver_serial_callback(std::string incoming);

    void f_command_thrust_loop();

    void f_read_config();

    void f_initialize_topics();

    void f_initialize_pwm_channels();

public:
    AlphaDriverRos();

    void initialize();

};


#endif //ALPHA_DRIVER_ALPHADRIVERROS_H
