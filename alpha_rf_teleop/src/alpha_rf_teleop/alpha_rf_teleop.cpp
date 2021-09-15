#include "alpha_rf_teleop.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Int16.h"
#include "rf_comms.h"
#include "nmea.h"
#include "geometry_msgs/Vector3Stamped.h"

RfTeleop::RfTeleop() :
    m_nh(""),
    m_pnh("~")
{
m_pnh.param<std::string>("port", m_port, "/dev/xbee");
    m_pnh.param<int>("baud", m_baud, 115200);

    m_comms = boost::make_shared<RfComms>(m_port, m_baud);

    m_nmea_callback = m_pnh.subscribe("rf/nmea", 10, &RfTeleop::f_nmea_callback, this);
    m_gps_callback = m_pnh.subscribe("rf/gps", 10, &RfTeleop::f_gps_callback, this);

    m_thrust_publisher = m_pnh.advertise<geometry_msgs::Vector3Stamped>("rf/thurst", 10);

    m_activate_getty_service = m_pnh.advertiseService("activate_getty", &RfTeleop::f_activate_getty, this);
    m_activate_teleop_service = m_pnh.advertiseService("add_two_ints", &RfTeleop::f_activate_teleop, this);

}

void RfTeleop::f_serial_callback(std::string incoming) {
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

    if (nmea_msg.command == NMEA_THRUST_PWM_REPORT){
        geometry_msgs::Vector3Stamped p;
        p.header.stamp = ros::Time::now();
        p.vector.x = nmea_msg.values[0];
        p.vector.y = nmea_msg.values[1];
        p.vector.z = nmea_msg.values[2];
        m_thrust_publisher.publish(p);
    }
}

void RfTeleop::f_gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    if(!m_teleop_active) {
        return;
    }

    NMEA data;

    data.construct("GPS",
        (float)msg->status.status,
        (float)msg->status.service,
        (float)msg->latitude,
        (float)msg->longitude,
        (float)msg->altitude
    );

    m_comms->sendLine(data.get_raw());
}

void RfTeleop::f_nmea_callback(const std_msgs::String::ConstPtr &msg) {
    m_comms->sendLine(msg->data);
}

bool RfTeleop::f_activate_getty(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    m_comms->deactivate();
    std::this_thread::sleep_for(std::chrono::seconds(3));

    execl("/usr/bin/sudo",
            "/usr/bin/sudo",
            "systemctl",
            "start",
            "serial-getty@xbee",
            nullptr
    );

    return true;
}

bool RfTeleop::f_activate_teleop(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    m_comms->activate();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    execl("/usr/bin/sudo",
          "/usr/bin/sudo",
          "systemctl",
          "stop",
          "serial-getty@xbee",
          nullptr
    );

    return true;
}