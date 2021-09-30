#include "alpha_rf_teleop_remote.h"

#include "alpha_rf_teleop.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Int16.h"
#include "rf_comms.h"
#include "nmea.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "chrono"
#include "thread"

RfRemote::RfRemote() :
        m_nh(""),
        m_pnh("~")
{
    m_pnh.param<std::string>("port", m_port, "/dev/ttyUSB0");
    m_pnh.param<int>("baud", m_baud, 115200);

    m_comms = boost::make_shared<RfComms>(m_port, m_baud);


    m_comms->setCallback(boost::bind(&RfRemote::f_serial_callback, this, boost::placeholders::_1));

    m_joy_teleop_callback = m_nh.subscribe("thrust_cmd", 10, &RfRemote::f_joy_teleop_callback, this);

    m_incoming_publisher = m_nh.advertise<std_msgs::String>("incoming", 10);
    m_gps_publisher = m_nh.advertise<sensor_msgs::NavSatFix>("fix", 10);

    m_comms->activate();
}

void RfRemote::f_joy_teleop_callback(const geometry_msgs::Vector3Stamped::ConstPtr &msg) {
    NMEA data;

    data.construct("%s,%d,%d,%d",NMEA_THRUST_PWM_CMD,
                   (int)msg->vector.x,
                   (int)msg->vector.y,
                   (int)msg->vector.z
    );

    m_comms->sendLine(data.get_raw());
}

void RfRemote::f_serial_callback(std::string incoming) {
    if(!ros::ok()) {
        return;
    }

    std_msgs::String d;
    d.data = incoming;

    ROS_WARN_STREAM(d);
    m_incoming_publisher.publish(d);

    std_msgs::String raw_msg;
    raw_msg.data = incoming;

    NMEA data;
    data.parse(incoming);

    if(not data.get_valid()) {
        return;
    }

    alpha_msgs::NMEA nmea_msg;

    nmea_msg.header.stamp = ros::Time::now();
    nmea_msg.command = std::string(data.get_cmd());
    nmea_msg.values = std::vector<float>(data.get_values(), data.get_values() + data.get_argc());

    if (nmea_msg.command == "GPS"){
        sensor_msgs::NavSatFix gps_msg;
        gps_msg.header.stamp = ros::Time::now();
        gps_msg.status.status = (uint)nmea_msg.values[0];
        gps_msg.status.service = (int)nmea_msg.values[1];
        gps_msg.latitude = nmea_msg.values[2];
        gps_msg.longitude = nmea_msg.values[3];
        gps_msg.altitude = nmea_msg.values[4];

        ROS_WARN_STREAM(gps_msg);
        m_gps_publisher.publish(gps_msg);
    }
}