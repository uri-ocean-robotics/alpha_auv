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
#include "mvp_msgs/NMEA.h"
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
