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

    m_comms->activate();

    m_comms->setCallback(boost::bind(&RfRemote::f_serial_callback, this, boost::placeholders::_1));

    m_joy_teleop_callback = m_nh.subscribe("thrust_cmd", 10, &RfRemote::f_joy_teleop_callback, this);

    m_incoming_publisher = m_nh.advertise<std_msgs::String>("incoming", 10);

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

    std_msgs::String d;
    d.data = incoming;

    m_incoming_publisher.publish(d);

    std_msgs::String raw_msg;
    raw_msg.data = incoming;

    NMEA data;
    data.parse(incoming.c_str());

    if(not data.get_valid()) {
        return;
    }

    alpha_msgs::NMEA nmea_msg;

    nmea_msg.header.stamp = ros::Time::now();
    nmea_msg.command = std::string(data.get_cmd());
    nmea_msg.values = std::vector<float>(data.get_values(), data.get_values() + data.get_argc());

    if (nmea_msg.command == "GPS"){
        sensor_msgs::NavSatFix m;
        m.header.stamp = ros::Time::now();
        m.status.status = nmea_msg.values[0];
        m.status.service = nmea_msg.values[1];
        m.latitude = nmea_msg.values[2];
        m.longitude = nmea_msg.values[3];
        m.altitude = nmea_msg.values[4];

        m_gps_publisher.publish(m);
    }
}